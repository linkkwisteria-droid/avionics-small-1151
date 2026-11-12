/*
Project: Small Avionics 1151
Location: ./firmware/avionics-small-1151/avonics-small-1151.ino
Author: Wisteria
Date: 2026-10-11
Summary: Main Firmware of Small Avionics 1151, USING FREERTOS
*/



// Head File

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>



/* Constant Definition */

// Serial, I2C
#define BAUD 115200
#define I2C_SDA 21
#define I2C_SCL 22

// PERIOD
#define SENSORS_PERIOD_MS 100
#define SD_PERIOD_MS 500
#define BUZZER_PERIOD_MS 2000

// BMP280
#define BMP280_I2C 0x76
#define MODE Adafruit_BMP280::MODE_NORMAL
#define TEMPSAMPLING Adafruit_BMP280::SAMPLING_X2
#define PRESSSAMPLING Adafruit_BMP280::SAMPLING_X16
#define FILTER Adafruit_BMP280::FILTER_X16
#define DURATION Adafruit_BMP280::STANDBY_MS_1

// MPU6050
#define MPU6050_RANGE_G MPU6050_RANGE_8_G
#define MPU6050_RANGE_DEG MPU6050_RANGE_500_DEG
#define MPU6050_BAND_HZ MPU6050_BAND_21_HZ

// SD

// Hatch
#define HATCH_BUZZER 17             // its functions depend on you
#define HATCH_LED_STATUS_PIN 16     // display fsm by sparkle
#define HATCH_LED_LIFTOFF_PIN 4     // display liftoff is high or low
#define HATCH_LIFTOFF_PIN 0         // high is on ground should not release parachute, low is flying can release parachute

// Recovery (release parachute)
#define RECOVERY_IGNITER_PIN 13     // high to ignite igniter to release parachute
#define RECOVERY_PWM_PIN 12         // pwm



/* Function Declaration */

// Setup
void serial_setup();    // setup serial and baud
void bmp280_setup();    // setup bmp280
void mpu6050_setup();   // setup mpu6050
void sd_setup();        // setup sd
void fsm_setup();       // fsm

// FreeRTOS Task
void TaskSensors(void *pvParameters);     // get data from sensors
void TaskSD(void *pvParameters);          // save data into sd
void TaskRecovery(void *pvParameters);    // using igniter and pwm(sever motor)
void TaskStatusLED(void *pvParameters);   // 2 led, defferent function
void TaskBuzzer(void *pvParameters);      // depend on you



/* Global Variable Declaration */

// Device Availability
bool serial_availability = false;
bool bmp280_availability = false;
bool mpu6050_availability = false;
bool sd_availability = false;
bool recovery_availability = true;



// Finite-State Machine
enum fsm {
  STANDBY = 0,
  LIFTOFF = 1,
  LANDING = 2,
  GROUND = 3
}



/* Setup */

void setup() {
  // Serial Setup
  serial_setup();
  if (serial_availability == true) {
    Serial.print(F"Serial is OK. Baud is");
    Serial.println(BAUD);
  }

  // Sensors Setup
  bmp280_setup();
  mpu6050_setup();
  if (serial_availability == true) {
    if (bmp280_availability) {
      
    }
    else {

    }
    if (mpu6050_availability) {

    }
    else {

    }
  }

  // SD Setup

  // Hatch Setup
  
  // Recovery Setup
}

void loop() {
  // put your main code here, to run repeatedly:

}
