/*
Project: Small Avionics 1151
Location: ./firmware/avionics-small-1151/avonics-small-1151.ino
Author: Wisteria, Gemini
Date: 2026-10-11
Summary: Main Firmware of Small Avionics 1151, USING FREERTOS
*/

#include <Wire.h>       
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>



/* Constant Definition */
// Serial, I2C
#define BAUD 115200
#define I2C_SDA 21
#define I2C_SCL 22
#define SPI_SD_CS 5

// FreeRTOS
#define WORK_PERIOD_MS 10
#define QUEUE_SIZE 100
#define QUEUE_BATCH_SIZE 20

// BMP280
#define BMP280_I2C 0x76
#define MODE Adafruit_BMP280::MODE_NORMAL
#define TEMP_SAMPLING Adafruit_BMP280::SAMPLING_X2
#define PRESS_SAMPLING Adafruit_BMP280::SAMPLING_X16
#define FILTER Adafruit_BMP280::FILTER_X16
#define STANDBY_MS Adafruit_BMP280::STANDBY_MS_500

// MPU6050
#define MPU6050_RANGE_G MPU6050_RANGE_8_G
#define MPU6050_RANGE_DEG MPU6050_RANGE_500_DEG
#define MPU6050_BAND_HZ MPU6050_BAND_21_HZ

// Hatch & Pins
#define HATCH_BUZZER 17             // 蜂鳴器腳位
#define HATCH_LED_STATUS_PIN 16     // 綠色 LED：待命、起飛、開傘、降落
#define HATCH_LED_LIFTOFF_PIN 4     // 紅色 LED：離架指示
#define HATCH_LIFTOFF_PIN 2         // 離架偵測開關腳位

// Recovery (release parachute)
#define RECOVERY_IGNITER_PIN 13     // 點火開傘高電位觸發腳位
#define RECOVERY_PWM_PIN 12         // 伺服馬達 PWM 腳位



/* Function Declaration */
// Setup
void serial_setup();
void bmp280_setup();
void mpu6050_setup();
void sd_setup();
void fsm_setup();       // 要從那個只有一個數字的檔案去取得最新狀態

// FUNCTIONs
void print_setup_result();
void update_state();

// FreeRTOS Task
void TaskSensors(void *pvParameters);     // 感測器、狀態機與序列埠繪圖
void TaskSD(void *pvParameters);          // SD 卡日誌記錄
void TaskRecovery(void *pvParameters);    // 點火開傘與狀態動作
void TaskIndicator(void *pvParameters);   // 



/* Global Variable Declaration */
// Availability
bool serial_availability = false;
bool bmp280_availability = false;
bool mpu6050_availability = false;
bool sd_availability = false;
bool recovery_availability = true;

// Device Declaration
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// Finite-State Machine 狀態機定義
enum FSMState {
  STANDBY = 0,
  ASCENT  = 1,
  DESCENT = 2,
  GROUND = 3
};

FSMState state = STANDBY;

// Sensor Data
typedef struct {
  unsigned long ms;
  int state;  
  float press;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} flight_data;

// FreeRTOS Synchronization
QueueHandle_t Queue;
TaskHandle_t updateStateHandle = NULL;
TaskHandle_t recoveryHandle = NULL;


/* Setup & Loop*/
void setup() {
  // 建立 Queue
  Queue = xQueueCreate(QUEUE_SIZE, sizeof(flight_data));

  // 1. Serial Setup
  serial_setup();

  // 2. Sensors Setup
  Wire.begin(I2C_SDA, I2C_SCL);
  bmp280_setup();
  mpu6050_setup();

  // 3. SD & State Setup
  sd_setup();
  fsm_setup();

  // 4. Serial Print Result
  print_setup_result();

  // 5. Hardware Pins Setup
  pinMode(HATCH_LED_STATUS_PIN, OUTPUT);
  pinMode(HATCH_LED_LIFTOFF_PIN, OUTPUT);
  pinMode(HATCH_BUZZER, OUTPUT);
  pinMode(HATCH_LIFTOFF_PIN, INPUT_PULLUP);
  pinMode(RECOVERY_IGNITER_PIN, OUTPUT);
  digitalWrite(RECOVERY_IGNITER_PIN, LOW);

  // 6. 建立 FreeRTOS 多任務
  xTaskCreate(TaskSensors, "SensorsTask", 4096, NULL, 5, NULL);
  xTaskCreate(TaskUpdateData, "UpdateDataTask", 8192, NULL, 2, NULL);  
  xTaskCreate(TaskUpdateState, "UpdateStateTask", 4096, NULL, 3, NULL);
  xTaskCreate(TaskRecovery, "RecoveryTask", 2048, NULL, 4, NULL);
  xTaskCreate(TaskIndicator, "IndicatorTask", 2048, NULL, 1, NULL);
}

void loop() {
  // 採行 FreeRTOS 架構，loop 保留空或微幅延遲即可
  vTaskDelay(pdMS_TO_TICKS(1000));
}



/* function */
// Setup
void serial_setup() {
  Serial.begin(BAUD);
  delay(1000);
  if (!Serial) serial_availability = false;
  else serial_availability = true;
}

void bmp280_setup() {
  if (!bmp.begin(BMP280_I2C)) bmp280_availability = false;
  else {
    bmp280_availability = true;
    bmp.setSampling(MODE, TEMP_SAMPLING, PRESS_SAMPLING, FILTER, STANDBY_MS);
  }
}


void mpu6050_setup() {
  if (!mpu.begin()) mpu6050_availability = false;
  else {
    mpu6050_availability = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_G);
    mpu.setGyroRange(MPU6050_RANGE_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_HZ);
  }
}

void sd_setup() {
  if (!SD.begin(SPI_SD_CS)) sd_availability = false;
  else sd_availability = true;
}

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

// Others
void print_setup_result() {
  if (serial_availability) {
    if (bmp280_availability) Serial.println(F("BMP280 is Successful."));
    else Serial.println(F("BMP280 is Failed."));
    if (mpu6050_availability) Serial.println(F("MPU6050 is Successful."));
    else Serial.println(F("MPU6050 is Failed."));
    if (sd_availability) Serial.println(F("SD is Successful."));
    else Serial.println(F("SD is Failed."));
    Serial.print(F("State is"));
    Serial.println(state);
  }
}

void update_state() {
  File file = SD.open("/state.txt", FILE_WRITE);
  if (file) {
    file.seek(0);
    file.print((int)state);
    file.close();
  }  
}



/* FreeRTOS Tasks Definition */

// 1. 感測器任務與狀態機運算
void TaskSensors(void *pvParameters) {
  TickType_t xTaskGetTick = xTaskGetTickCount();
  sensors_event_t a, g, temp;

  while (1) {
    // 讀取感測器 預設都為0
    flight_data data = {
      .ms = pdTICKS_TO_MS(xTaskGetTick), .state = (int)state, .press = 0,
      .ax = 0, .ay = 0, .az = 0,
      .gx = 0, .gy = 0, .gz = 0
    };
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

    
    // 將數據傳送到 SD 佇列
    xQueueSend(Queue, &data, 0);

    // 狀態機判斷 若有改變則通知另一個task去sd卡儲存最新狀態
    int last_state = state;
    switch (state) {
      case STANDBY:
        if (data.az > 15) {
          state = ASCENT;
        }
        break;
      case ASCENT:
        if (data.az < 0) {
          state = DESCENT;
        }
        break;
      case DESCENT:
        if (data.az < -9) {
          state = GROUND;
        }
        break;
      case GROUND:
        break;
    }
    if (last_state != state) {
      if (updateStateHandle != NULL) {
        xTaskNotifyGive(updateStateHandle);
      }
    }

    //判斷開傘
    if (last_state == ASCENT && state == DESCENT) {
      if (recoveryHandle != NULL) {
        xTaskNotifyGive(recoveryHandle);
      }
    }

    // Serial Data
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

    vTaskDelayUntil(&xTaskGetTick, pdMS_TO_TICKS(WORK_PERIOD_MS));
  }
}

// 2. SD 卡記錄任務
void TaskUpdateData(void *pvParameters) {
  while (1) {
    UBaseType_t items_count = uxQueueMessagesWaiting(Queue);
    if (items_count >= (QUEUE_BATCH_SIZE)) {
      flight_data buffer[QUEUE_BATCH_SIZE];
      // 從 Queue 中把資料批量拿出來
      int count = 0;
      while (count < QUEUE_BATCH_SIZE && xQueueReceive(Queue, &buffer[count], 0) == pdTRUE) {
        count++;
      }

      // 如果成功拿到資料，且 SD 卡可用，則寫入 CSV
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
            dataFile.print("\n");     //必須這樣寫否則你的什麼東西會超出容量就會整個不能寫資料進去sd這非常重要我花了4小時處理這裡最後才知道要這樣寫
          }
        }
        dataFile.close();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(WORK_PERIOD_MS));
  }
}

// 3. 更新最新狀態進入SD卡 
void TaskUpdateState(void *pvParameters) {
  updateStateHandle = xTaskGetCurrentTaskHandle();
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    File file = SD.open("/state.txt", FILE_WRITE);
    if (file) {
      file.seek(0);
      file.print((int)state);
      file.close();
    }  
  }
}


// 4. 點火與開傘控制任務
void TaskRecovery(void *pvParameters) {
  recoveryHandle = xTaskGetCurrentTaskHandle();
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (recovery_availability == true) {
      recovery_availability = false; // 避免重複觸發
      Serial.println(F("[Recovery] Triggering igniter and parachute release!"));
      if (digitalRead(HATCH_LIFTOFF_PIN) == HIGH) {
        digitalWrite(RECOVERY_IGNITER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 點火維持 1 秒
        digitalWrite(RECOVERY_IGNITER_PIN, LOW);
      }
    }
  }
}

// 5. indicator 閃爍任務（待命、起飛、開傘、降落各有不同頻率）
void TaskIndicator(void *pvParameters) {
  while (1) {
    int delay_ms = 1000;
    switch (state) {
      case STANDBY:  delay_ms = 1000; break; // 慢閃 (1Hz)
      case ASCENT:  delay_ms = 100;  break; // 快閃 (10Hz)
      case DESCENT: delay_ms = 500;  break; // 極快閃 (2Hz)
      case GROUND:  delay_ms = 2000;  break; // 中閃 (0.5Hz)
    }

    digitalWrite(HATCH_LED_STATUS_PIN, HIGH);
    digitalWrite(HATCH_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));
    digitalWrite(HATCH_LED_STATUS_PIN, LOW);
    digitalWrite(HATCH_BUZZER, LOW);
    vTaskDelay(pdMS_TO_TICKS(delay_ms / 2));

    // liftoff led
    if (digitalRead(HATCH_LIFTOFF_PIN) == HIGH) digitalWrite(HATCH_LED_LIFTOFF_PIN, HIGH);
    else digitalWrite(HATCH_LED_LIFTOFF_PIN, LOW);
  }
}

