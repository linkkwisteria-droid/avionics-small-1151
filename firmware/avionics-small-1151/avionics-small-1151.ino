/*
Project: Small Avionics 1151 (小航電 1151 飛行電腦韌體)
Location: ./firmware/avionics-small-1151/avonics-small-1151.ino
Author: Wisteria, Gemini
Date: 2026-10-11
Summary: Main Firmware of Small Avionics 1151, USING FREERTOS
Description: 採用 FreeRTOS 多任務架構的小火箭航電韌體，負責高頻率感測器採集、
             即時飛行狀態機（FSM）判定、SD卡批量日誌記錄、點火開傘控制與狀態指示。
*/

#include <Wire.h>         // I2C 通訊庫（用於連接 BMP280 與 MPU6050）
#include <SPI.h>          // SPI 通訊庫（用於 MicroSD 卡模組）
#include <Adafruit_BMP280.h> // 氣壓計/溫度感測器庫
#include <Adafruit_MPU6050.h>// 6軸慣導感測器（加速度計與陀螺儀）庫
#include <Adafruit_Sensor.h> // Adafruit 統一感測器抽象層
#include <SD.h>           // SD卡檔案系統操作庫



/* =========================================================================
 * 1. 系統常數與硬體接腳定義 (Constant Definition)
 * ========================================================================= */
// 序列埠與通訊匯流排設定
#define BAUD 115200
#define I2C_SDA 21        // ESP32 SDA 腳位
#define I2C_SCL 22        // ESP32 SCL 腳位
#define SPI_SD_CS 5       // SD 卡晶片選擇 (Chip Select) 腳位

// FreeRTOS 核心調度參數
#define WORK_PERIOD_MS 10     // 感測器採集任務週期：10ms (100Hz 高頻採樣)
#define QUEUE_SIZE 100        // 佇列最大容量（儲存 flight_data 結構體數量）
#define QUEUE_BATCH_SIZE 20   // SD卡批次寫入閾值：累積 20 筆資料再一次性寫入，減少 IO 阻塞抖動

// BMP280 氣壓計硬體設定（追求高精度與抗雜訊）
#define BMP280_I2C 0x76       // I2C 位址 (通常為 0x76 或 0x77)
#define MODE Adafruit_BMP280::MODE_NORMAL // 正常量測模式
#define TEMP_SAMPLING Adafruit_BMP280::SAMPLING_X2    // 溫度過取樣 2x
#define PRESS_SAMPLING Adafruit_BMP280::SAMPLING_X16 // 氣壓過取樣 16x（提高高度計算平滑度）
#define FILTER Adafruit_BMP280::FILTER_X16           // IIR 濾波器係數 16
#define STANDBY_MS Adafruit_BMP280::STANDBY_MS_500   // 待命間隔

// MPU6050 慣導感測器設定
#define MPU6050_RANGE_G MPU6050_RANGE_8_G       // 加速度量程 ±8g（涵蓋火箭起飛高過載）
#define MPU6050_RANGE_DEG MPU6050_RANGE_500_DEG // 陀螺儀量程 ±500 deg/s
#define MPU6050_BAND_HZ MPU6050_BAND_21_HZ      // 數位低通濾波器頻寬 21Hz

// 艙體控制與指示燈、蜂鳴器腳位
#define HATCH_BUZZER 17             // 蜂鳴器腳位（提供狀態音效提示）
#define HATCH_LED_STATUS_PIN 16     // 狀態指示綠色 LED：依 FSM 狀態改變閃爍頻率
#define HATCH_LED_LIFTOFF_PIN 4     // 離架指示紅色 LED：指示火箭是否脫離發射架
#define HATCH_LIFTOFF_PIN 2         // 機械式離架偵測開關腳位 (Input Pullup)

// 航電回收與點火機構 (Parachute Recovery)
#define RECOVERY_IGNITER_PIN 13     // 電子點火開傘高電位觸發腳位（驅動 MOSFET）
#define RECOVERY_PWM_PIN 12         // 伺服馬達 PWM 腳位（備用機械脫落機構）



/* =========================================================================
 * 2. 函式宣告 (Function Declaration)
 * ========================================================================= */
// 初始化相關函式
void serial_setup();
void bmp280_setup();
void mpu6050_setup();
void sd_setup();
void fsm_setup();       // 從 SD 卡讀取上次中斷時的飛行狀態，實現防重啟丟失

// 系統輔助函式
void print_setup_result();
void update_state();

// FreeRTOS 任務宣告（各司其職，維持獨立的時間週期與優先權）
void TaskSensors(void *pvParameters);     // 任務 1：高頻感測器採集、FSM 狀態判定與即時序列埠輸出
void TaskUpdateData(void *pvParameters);  // 任務 2：中頻 SD 卡日誌批量寫入（避免 IO 卡死主控迴圈）
void TaskUpdateState(void *pvParameters); // 任務 3：事件驅動式狀態持久化（透過 Task Notification 觸發）
void TaskRecovery(void *pvParameters);    // 任務 4：事件驅動式點火與開傘控制
void TaskIndicator(void *pvParameters);   // 任務 5：低優先權狀態指示燈與蜂鳴器閃爍控制



/* =========================================================================
 * 3. 全域變數與資料結構宣告 (Global Variable Declaration)
 * ========================================================================= */
// 硬體模組可用性狀態旗標
bool serial_availability = false;
bool bmp280_availability = false;
bool mpu6050_availability = false;
bool sd_availability = false;
bool recovery_availability = true; // 點火保險旗標（確保只會觸發一次開傘）

// 感測器硬體物件實例
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// 有限狀態機（FSM）列舉定義：記錄火箭當前飛行階段
enum FSMState {
  STANDBY = 0,  // 待命狀態：靜止於發射架上
  ASCENT  = 1,  // 上升狀態：偵測到起飛高加速度 (Az > 15g)
  DESCENT = 2,  // 下降狀態：偵測到速度轉折/極點（Apogee），準備展傘
  GROUND  = 3   // 降落落地：觸地完成
};

FSMState state = STANDBY;

// 飛行數據封包結構體（用於 Queue 傳遞與 SD 批量紀錄）
typedef struct {
  unsigned long ms; // 系統運行毫秒數
  int state;        // 當前狀態機數值
  float press;      // 氣壓值
  float ax, ay, az; // 三軸加速度 (m/s^2 或 g)
  float gx, gy, gz; // 三軸角速度
} flight_data;

// FreeRTOS 同步控制代碼 (Handles)
QueueHandle_t Queue;                      // 傳遞 flight_data 的安全佇列
TaskHandle_t updateStateHandle = NULL;    // TaskUpdateState 的任務控制代碼（用於接收通知）
TaskHandle_t recoveryHandle = NULL;       // TaskRecovery 的任務控制代碼（用於接收通知）



/* =========================================================================
 * 4. 系統初始化與主迴圈 (Setup & Loop)
 * ========================================================================= */
void setup() {
  // 建立 FreeRTOS 執行緒安全佇列（容量 QUEUE_SIZE，每格大小為 flight_data 結構體）
  Queue = xQueueCreate(QUEUE_SIZE, sizeof(flight_data));

  // 1. 初始化 Serial 通訊
  serial_setup();

  // 2. 初始化 I2C 匯流排與感測器 (BMP280, MPU6050)
  Wire.begin(I2C_SDA, I2C_SCL);
  bmp280_setup();
  mpu6050_setup();

  // 3. 初始化 SD 卡與讀取先前狀態機
  sd_setup();
  fsm_setup();

  // 4. 序列埠印出各項硬體初始化檢測結果
  print_setup_result();

  // 5. 硬體腳位模式設定 (PinMode)
  pinMode(HATCH_LED_STATUS_PIN, OUTPUT);
  pinMode(HATCH_LED_LIFTOFF_PIN, OUTPUT);
  pinMode(HATCH_BUZZER, OUTPUT);
  pinMode(HATCH_LIFTOFF_PIN, INPUT_PULLUP); // 啟用內部上拉電阻（離架開關）
  pinMode(RECOVERY_IGNITER_PIN, OUTPUT);
  digitalWrite(RECOVERY_IGNITER_PIN, LOW);   // 預設關閉點火訊號，確保安全

  // 6. 建立 FreeRTOS 多任務（依任務重要性分配 Stack 大小與優先權 Priority）
  // 參數依序為：任務函數, 任務名稱, Stack大小(bytes), 傳入參數, 優先權(數字越大越高), 任務Handle
  xTaskCreate(TaskSensors, "SensorsTask", 4096, NULL, 5, NULL);          // 優先權最高：高頻採集與姿態判定
  xTaskCreate(TaskUpdateData, "UpdateDataTask", 8192, NULL, 2, NULL);   // SD卡日誌記錄
  xTaskCreate(TaskUpdateState, "UpdateStateTask", 4096, NULL, 3, NULL); // 狀態變化同步
  xTaskCreate(TaskRecovery, "RecoveryTask", 2048, NULL, 4, NULL);       // 點火開傘控制（次高優先權）
  xTaskCreate(TaskIndicator, "IndicatorTask", 2048, NULL, 1, NULL);     // 狀態指示燈（最低優先權）
}

void loop() {
  // 採用 FreeRTOS 搶佔式排程架構後，Arduino 標準的 loop 函數只需保持微幅延遲或留空，
  // 所有實際工作已交由上述獨立運行的 FreeRTOS Tasks 處理。
  vTaskDelay(pdMS_TO_TICKS(1000));
}



/* =========================================================================
 * 5. 初始化輔助函數實作 (Setup Helper Functions)
 * ========================================================================= */
void serial_setup() {
  Serial.begin(BAUD);
  delay(1000);
  if (!Serial) serial_availability = false;
  else serial_availability = true;
}

void bmp280_setup() {
  if (!bmp.begin(BMP280_I2C)) {
    bmp280_availability = false;
  } else {
    bmp280_availability = true;
    // 套用高精度氣壓計採樣與過濾設定
    bmp.setSampling(MODE, TEMP_SAMPLING, PRESS_SAMPLING, FILTER, STANDBY_MS);
  }
}

void mpu6050_setup() {
  if (!mpu.begin()) {
    mpu6050_availability = false;
  } else {
    mpu6050_availability = true;
    mpu6050_setup_ranges:
    mpu.setAccelerometerRange(MPU6050_RANGE_G);
    mpu.setGyroRange(MPU6050_RANGE_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_HZ);
  }
}

void sd_setup() {
  if (!SD.begin(SPI_SD_CS)) {
    sd_availability = false;
  } else {
    sd_availability = true;
  }
}

// 從 SD 卡讀取系統最後狀態，若檔案不存在則建立預設的 STANDBY 狀態
void fsm_setup() {
  if (SD.exists("/state.txt")) {
    File file = SD.open("/state.txt", FILE_READ);
    if (file) {
      int value = file.parseInt();
      state = (FSMState)value;
      file.close();
    }
  } else {
    File file = SD.open("/state.txt", FILE_WRITE);
    if (file) {
      file.print((int)STANDBY);
      file.close();
    }
  }
}

// 序列埠印出硬體自我檢測報告
void print_setup_result() {
  if (serial_availability) {
    if (bmp280_availability) Serial.println(F("BMP280 is Successful."));
    else Serial.println(F("BMP280 is Failed."));
    
    if (mpu6050_availability) Serial.println(F("MPU6050 is Successful."));
    else Serial.println(F("MPU6050 is Failed."));
    
    if (sd_availability) Serial.println(F("SD is Successful."));
    else Serial.println(F("SD is Failed."));
    
    Serial.print(F("State is "));
    Serial.println(state);
  }
}

// 更新狀態到 SD 卡的輔助函式（直接覆寫開頭）
void update_state() {
  File file = SD.open("/state.txt", FILE_WRITE);
  if (file) {
    file.seek(0);
    file.print((int)state);
    file.close();
  }  
}



/* =========================================================================
 * 6. FreeRTOS 任務核心實作 (FreeRTOS Tasks Definition)
 * ========================================================================= */

// --- 任務 1：高頻感測器採集與飛行狀態機（FSM）判定 ---
void TaskSensors(void *pvParameters) {
  TickType_t xTaskGetTick = xTaskGetTickCount();
  sensors_event_t a, g, temp;

  while (1) {
    // 初始化當前數據結構（預設值為 0）
    flight_data data = {
      .ms = pdTICKS_TO_MS(xTaskGetTick), .state = (int)state, .press = 0,
      .ax = 0, .ay = 0, .az = 0,
      .gx = 0, .gy = 0, .gz = 0
    };

    // 若感測器可用，讀取即時數值
    if (bmp280_availability == true) data.press = bmp.readPressure();
    if (mpu6050_availability == true) {
      mpu.getEvent(&a, &g, &temp);
      data.ax = a.acceleration.x;
      data.ay = a.acceleration.y;
      data.az = a.acceleration.z;
      data.gx = g.gyro.x;
      data.gy = g.gyro.y;
      data.gz = g.gyro.z;
    }

    // 將採集到的完整數據包推送到 Queue 中（等待時間為 0，若滿則不阻塞直接跳過）
    xQueueSend(Queue, &data, 0);

    // 有限狀態機（FSM）邏輯判定
    int last_state = state;
    switch (state) {
      case STANDBY:
        // 偵測發射點火起飛：當垂直加速度 Az 大於 15g 時進入上升期
        if (data.az > 15) {
          state = ASCENT;
        }
        break;
      
      case ASCENT:
        // 偵測極點（Apogee）：當加速度轉為負值（自由落體或減速轉折）時判定進入下降段
        if (data.az < 0) {
          state = DESCENT;
        }
        break;
      
      case DESCENT:
        // 偵測落地：當降落傘張開後垂直減速或衝擊力達到特定閾值
        if (data.az < -9) {
          state = GROUND;
        }
        break;
      
      case GROUND:
        // 落地後狀態保持，不需變更
        break;
    }

    // 若 FSM 狀態發生改變，透過 Task Notification 喚醒狀態寫入任務
    if (last_state != state) {
      if (updateStateHandle != NULL) {
        xTaskNotifyGive(updateStateHandle);
      }
    }

    // 關鍵轉折點偵測：由 ASCENT 進入 DESCENT 時，觸發開傘與點火機制
    if (last_state == ASCENT && state == DESCENT) {
      if (recoveryHandle != NULL) {
        xTaskNotifyGive(recoveryHandle);
      }
    }

    // 將數據透過 Serial 輸出（可用於地面站即時繪圖或序列埠監控）
    if (serial_availability) {
      Serial.print(F("Ms:"));        
      Serial.print(data.ms);
      Serial.print(F(","));
      Serial.print(F("State:"));        
      Serial.print((int)data.state);
      Serial.print(F(","));
      Serial.print(F("Press:"));
      Serial.print(data.press);
      Serial.print(F(","));
      Serial.print(F("AccX:"));
      Serial.print(data.ax);
      Serial.print(F(","));
      Serial.print(F("AccY:"));
      Serial.print(data.ay);
      Serial.print(F(","));
      Serial.print(F("AccZ:"));
      Serial.print(data.az);
      Serial.print(F(","));
      Serial.print(F("GyroX:"));
      Serial.print(data.gx);
      Serial.print(F(","));
      Serial.print(F("GyroY:"));
      Serial.print(data.gy);
      Serial.print(F(","));
      Serial.print(F("GyroZ:"));
      Serial.print(data.gz);
      Serial.println("");
    }

    // 使用絕對延時維持精準的 100Hz 採樣週期，避免累積誤差
    vTaskDelayUntil(&xTaskGetTick, pdMS_TO_TICKS(WORK_PERIOD_MS));
  }
}

// --- 任務 2：SD 卡批量日誌記錄任務 ---
void TaskUpdateData(void *pvParameters) {
  while (1) {
    // 檢查 Queue 中累積的資料量是否達到批次寫入閾值 (QUEUE_BATCH_SIZE)
    UBaseType_t items_count = uxQueueMessagesWaiting(Queue);
    if (items_count >= (QUEUE_BATCH_SIZE)) {
      flight_data buffer[QUEUE_BATCH_SIZE];
      int count = 0;
      
      // 從 Queue 中批量取出資料存入區域緩衝區
      while (count < QUEUE_BATCH_SIZE && xQueueReceive(Queue, &buffer[count], 0) == pdTRUE) {
        count++;
      }

      // 如果成功取得資料且 SD 卡可用，執行批次寫入 CSV
      if (count > 0 && sd_availability) {
        File dataFile = SD.open("/flight_log.csv", FILE_APPEND);
        for (int i = 0; i < count; i++) {
          if (dataFile) {
            dataFile.print(buffer[i].ms);
            dataFile.print(",");
            dataFile.print((int)buffer[i].state);
            dataFile.print(",");
            dataFile.print(buffer[i].press);
            dataFile.print(",");
            dataFile.print(buffer[i].ax);
            dataFile.print(",");
            dataFile.print(buffer[i].ay);
            dataFile.print(",");
            dataFile.print(buffer[i].az);
            dataFile.print(",");
            dataFile.print(buffer[i].gx);
            dataFile.print(",");
            dataFile.print(buffer[i].gy);
            dataFile.print(",");
            dataFile.print(buffer[i].gz);
            dataFile.print("\n"); // 採用明確的換行字元，防止 SD 卡寫入緩衝溢位導致整個檔案損毀
          }
        }
        dataFile.close();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(WORK_PERIOD_MS));
  }
}

// --- 任務 3：狀態持久化更新任務 (事件驅動) ---
void TaskUpdateState(void *pvParameters) {
  updateStateHandle = xTaskGetCurrentTaskHandle();
  while (1) {
    // 進入阻塞狀態，等待來自 TaskSensors 的 Task Notification 通知（省 CPU 資源）
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 當狀態改變時，將最新狀態寫入 SD 卡的 state.txt
    File file = SD.open("/state.txt", FILE_WRITE);
    if (file) {
      file.seek(0);
      file.print((int)state);
      file.close();
    }  
  }
}

// --- 任務 4：點火與開傘控制任務 (事件驅動) ---
void TaskRecovery(void *pvParameters) {
  recoveryHandle = xTaskGetCurrentTaskHandle();
  while (1) {
    // 等待頂點到達（ASCENT -> DESCENT）的任務通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    if (recovery_availability == true) {
      recovery_availability = false; // 鎖定保險，防止重複觸發點火訊號
      Serial.println(F("[Recovery] Triggering igniter and parachute release!"));
      
      // 透過實體離架開關確認火箭已經真正離架，增添雙重安全防護
      if (digitalRead(HATCH_LIFTOFF_PIN) == HIGH) {
        digitalWrite(RECOVERY_IGNITER_PIN, HIGH); // 啟動點火輸出
        vTaskDelay(pdMS_TO_TICKS(1000));           // 點火電流維持 1 秒以確保 e-match 順利引燃
        digitalWrite(RECOVERY_IGNITER_PIN, LOW);  // 關閉點火輸出
      }
    }
  }
}

// --- 任務 5：指示燈與蜂鳴器狀態頻率閃爍任務 ---
void TaskIndicator(void *pvParameters) {
  while (1) {
    int delay_ms = 1000;
    
    // 根據當前飛行狀態調整蜂鳴器與 LED 的閃爍頻率（便於地面觀察辨識）
    switch (state) {
      case STANDBY:  delay_ms = 1000; break; // 待命狀態：慢閃 (1Hz)
      case ASCENT:   delay_ms = 100;  break; // 上升狀態：急促快閃 (10Hz)
      case DESCENT:  delay_ms = 500;  break; // 下降開傘：中速閃爍 (2Hz)
      case GROUND:   delay_ms = 2000; break; // 落地狀態：超慢閃 (0.5Hz)
    }

    // 執行閃爍動作
    digitalWrite(HATCH_LED_STATUS_PIN, HIGH);
    digitalWrite(HATCH_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));
    digitalWrite(HATCH_LED_STATUS_PIN, LOW);
    digitalWrite(HATCH_BUZZER, LOW);
    vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));

    // 即時監控離架開關狀態並點亮對應的離架指示燈
    if (digitalRead(HATCH_LIFTOFF_PIN) == HIGH) {
      digitalWrite(HATCH_LED_LIFTOFF_PIN, HIGH);
    } else {
      digitalWrite(HATCH_LED_LIFTOFF_PIN, LOW);
    }
  }
}