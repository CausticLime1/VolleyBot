#include <cmath>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <queue.h>

//Digital Pins
#define BUZZER_PIN PB8
#define LED_PIN PB12

//Motor Pins
#define M1_F PA0
#define M1_B PA1
#define M2_F PA2
#define M2_B PA3
#define M3_F PA6
#define M3_B PA7

//Battery Variables
#define VREF 3.3
#define BATT_STAT PB0
#define BATT_VOLT PB1

//IMU Variables
#define IMU_ADDR 0x68
#define IMU_SDA PB7
#define IMU_CLK PB6

struct IMUData{
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t time;
};

float axCali = 0, ayCali = 0, azCali = 0, gxCali, gyCali, gzCali;

SemaphoreHandle_t i2cMutex;
QueueHandle_t imuQueue;

//Startup Beeps 
int startupNotes[] = {392, 587};

struct BeepCommand{
  int frequency;
  int durationMs;
};

QueueHandle_t beepQueue;

void setup(){
  Serial.begin(115200);
  //Motor Pin Setup
  pinMode(M1_F, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M2_B, OUTPUT);
  pinMode(M3_F, OUTPUT);
  pinMode(M3_B, OUTPUT);

  //Buzzer and LED Pin
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  //BATT Pins
  pinMode(BATT_STAT, INPUT);
  pinMode(BATT_VOLT, INPUT);

  //Setting Up IMU
  Wire.setSCL(IMU_CLK);
  Wire.setSDA(IMU_SDA);

  Wire.begin();
  Wire.setClock(400000);

  i2cMutex = xSemaphoreCreateMutex();

  imuQueue = xQueueCreate(1, sizeof(IMUData));
  beepQueue = xQueueCreate(10, sizeof(BeepCommand));

  xTaskCreate(vStartupTask, "Start", 512, NULL, 4, NULL);

  vTaskStartScheduler();
}

void vStartupTask(void *pvParameters){
  Wire.begin();

  long sumGx = 0, sumGy = 0, sumGz = 0;

  int samples = 500;
  for(int i = 0; i < samples; i++){
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 6);
    if (Wire.available() == 6){
      sumGx += (int16_t)(Wire.read() << 8 | Wire.read());
      sumGy += (int16_t)(Wire.read() << 8 | Wire.read());
      sumGz += (int16_t)(Wire.read() << 8 | Wire.read()); 
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  gxCali = (float)sumGx / samples;
  gyCali = (float)sumGy / samples;
  gzCali = (float)sumGz / samples;

  xTaskCreate(vIMUTask, "IMU", 512, NULL, 3, NULL);
  xTaskCreate(vDebugTask, "DBUG", 512, NULL, 1, NULL);
  xTaskCreate(vBuzzerTask, "Buzz", 256, NULL, 1, NULL);
  startupBeeps();
  vTaskDelete(NULL);
}

void startupBeeps(){
  BeepCommand beep1 = {392, 500};
  BeepCommand beep2 = {587, 500};
  xQueueSend(beepQueue, &beep1, pdMS_TO_TICKS(10));
  xQueueSend(beepQueue, &beep2, pdMS_TO_TICKS(10));
}

void vBuzzerTask(void *pvParameters) {
    BeepCommand cmd;
    for (;;) {
        if (xQueueReceive(beepQueue, &cmd, portMAX_DELAY)) {
            tone(BUZZER_PIN, cmd.frequency);
            vTaskDelay(pdMS_TO_TICKS(cmd.durationMs)); 
            noTone(BUZZER_PIN);
            
            // Short silence between beeps so they don't bleed together
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
    }
}

void vIMUTask(void *pvParameters){
  IMUData curIMUData;
  int16_t rawAx, rawAy, rawAz, temp, rawGx, rawGy, rawGz;
  while(true){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool dataReady = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))){
      Wire.beginTransmission(IMU_ADDR);
      Wire.write(0x3B);
      if (Wire.endTransmission() == 0 && Wire.requestFrom(IMU_ADDR, (uint8_t)14) == 14){
        rawAx = (Wire.read() << 8) | Wire.read();
        rawAy = (Wire.read() << 8) | Wire.read();
        rawAy = (Wire.read() << 8) | Wire.read();
        temp = (Wire.read() << 8) | Wire.read();
        rawGx = (Wire.read() << 8) | Wire.read();
        rawGy = (Wire.read() << 8) | Wire.read();
        rawGz = (Wire.read() << 8) | Wire.read();

        dataReady = true;
      }
      xSemaphoreGive(i2cMutex);
    }
    if (dataReady){
      curIMUData.ax = ((float)rawAx / 16384.0) - axCali;
      curIMUData.ay = ((float)rawAy / 16384.0) - ayCali;
      curIMUData.az = ((float)rawAz / 16384.0) - azCali;
      curIMUData.gx = ((float)rawGx / 131.0) - gxCali;
      curIMUData.gy = ((float)rawGy / 131.0) - gyCali;
      curIMUData.gz = ((float)rawGz / 131.0) - gzCali;

      xQueueOverwrite(imuQueue, &curIMUData);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

void vDebugTask(void *pvParameters) {
    IMUData displayData;

    for (;;) {
        // xQueuePeek looks at the data without removing it from the queue.
        // This allows the Motor Task to still get the same data later.
        if (xQueuePeek(imuQueue, &displayData, portMAX_DELAY)) {
            
            // Format for Serial Plotter (Label:Value)
            Serial.print("AccX:"); Serial.print(displayData.ax); Serial.print(",");
            Serial.print("AccY:"); Serial.print(displayData.ay); Serial.print(",");
            Serial.print("AccZ:"); Serial.print(displayData.az); Serial.print(",");
            
            Serial.print("GyroX:"); Serial.print(displayData.gx); Serial.print(",");
            Serial.print("GyroY:"); Serial.print(displayData.gy); Serial.print(",");
            Serial.print("GyroZ:"); Serial.println(displayData.gz); 
        }

        // Print 10 times per second (100ms delay)
        // This prevents the Serial buffer from choking.
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void loop(){
}