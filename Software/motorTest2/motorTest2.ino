#include <cmath>

//Digital Pins
#define BUZZER_PIN PB8
#define LED_PIN PB12
#define IMU_SDA PB7
#define IMU_CLK PB6

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

//Timing Variables
const int loopTimer = 1000;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

void startupBuzz(){
  static int step = 0;
  static long lastChange = 0;
  currentMillis = millis();

  switch (step){
    case 0:
      tone(BUZZER_PIN, 392);
      step = 1;
      lastChange = currentMillis;

    case 1:
      if (currentMillis - lastChange >= 400){
        noTone(BUZZER_PIN);
        lastChange = currentMillis;
        step = 2;
      }
    case 2:
      if (currentMillis - lastChange >= 200){
        tone(BUZZER_PIN, 587);
        lastChange = currentMillis;
        step = 3;
      }
    case 3:
      if (currentMillis - lastChange >= 500) {
        noTone(BUZZER_PIN);
        step = 4; 
      }
      break;
    case 4: 
      break;
  }
}

void motorStartupTest(){
  static int motorStep = 0;
  currentMillis = millis();
  static int motorLastChange = 0;

  switch (motorStep){
    case 0:
      analogWrite(M1_F, 128);
      analogWrite(M2_F, 128);
      analogWrite(M3_F, 128);
      motorStep = 1;
    case 1:
      if (currentMillis - motorLastChange > 5000){
        analogWrite(M1_F, 0);
        analogWrite(M2_F, 0);
        analogWrite(M3_F, 0);
        motorStep = 2;
      }
    case 2:
      break;
  }
  
}

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_C5  523

int melody[] = {
  NOTE_C4, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_D4,
  NOTE_D4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_C4,
  NOTE_C4, NOTE_A4, NOTE_A4, NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_D4,
  NOTE_C4, NOTE_C4, NOTE_D4, NOTE_G4, NOTE_E4, NOTE_F4
};

int durations[] = {
  4, 4, 8, 8, 8, 8, 4, 4,
  4, 4, 8, 8, 8, 8, 4, 4,
  4, 4, 8, 8, 8, 8, 4, 4,
  8, 8, 4, 4, 4, 2
};

int totalNotes = sizeof(melody) / sizeof(int);

void updateStartupSequence() {
  static int state = 0;             
  static int noteIndex = 0;          
  static unsigned long lastChange = 0;
  static bool isNotePlaying = false; // The flag to prevent repeating calls
  unsigned long now = millis();

  int noteDuration; 

  switch (state) {
    case 0: // Startup Tone 1
      if (!isNotePlaying) {
        tone(BUZZER_PIN, 392);
        isNotePlaying = true;
        lastChange = now;
      }
      if (now - lastChange >= 200) {
        noTone(BUZZER_PIN);
        isNotePlaying = false; // Reset for next state
        lastChange = now;
        state = 1;
      }
      break;

    case 1: // Startup Tone 2
      if (!isNotePlaying) {
        tone(BUZZER_PIN, 587);
        isNotePlaying = true;
        lastChange = now;
      }
      if (now - lastChange >= 500) {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
        isNotePlaying = false; // Reset for next state
        lastChange = now;
        state = 2; 
      }
      break;

    case 2: // The 1-Second Wait
      if (now - lastChange >= 1000) {
        lastChange = now;
        state = 3; 
      }
      break;

    case 3: // Christmas Song
      noteDuration = 1000 / durations[noteIndex]; 
      
      // Only start the note if it's not already playing
      if (!isNotePlaying) {
        tone(BUZZER_PIN, melody[noteIndex]);
        isNotePlaying = true;
        lastChange = now;
      }

      // Logic to stop the note after duration
      if (now - lastChange >= noteDuration) {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
        
        // Wait for a small "silence" gap (30% of note duration)
        if (now - lastChange >= (noteDuration * 1.3)) {
          isNotePlaying = false; // Allow next note to start
          noteIndex++;
          lastChange = now;
          if (noteIndex >= totalNotes) state = 4;
        }
      }
      break;

    case 4: // Finished
      noTone(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }
}

void updateLEDTimer(unsigned long onTime, unsigned long offTime, int totalBlinks) {
  static int state = 0;              // 0: Idle, 1: LED ON, 2: LED OFF
  static unsigned long lastChange = 0;
  static int currentBlinks = 0;
  unsigned long now = millis();

  switch (state) {
    case 0: // Idle state - wait for a reason to blink
      if (currentBlinks < totalBlinks) {
        state = 1; 
        lastChange = now; 
      }
      break;

    case 1: // LED is ON
      digitalWrite(LED_PIN, HIGH);
      if (now - lastChange >= onTime) {
        lastChange = now;
        state = 2; // Move to the OFF portion of the blink
      }
      break;

    case 2: // LED is OFF
      digitalWrite(LED_PIN, LOW); // Ensure pin is pulled to 0V
      if (now - lastChange >= offTime) {
        lastChange = now;
        currentBlinks++; // We finished one full blink cycle
        
        if (currentBlinks >= totalBlinks) {
          state = 0; // Finished all blinks, go back to Idle
          // Safety: ensure it stays off
          digitalWrite(LED_PIN, LOW); 
        } else {
          state = 1; // Start the next blink
        }
      }
      break;
  }
}

void setup(){

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
}

float batteryVoltage = 0; 
int batteryChargeStatus = 0;

void loop(){
  currentMillis = millis();
  updateStartupSequence();
  if (currentMillis - previousMillis >= loopTimer){
    previousMillis = currentMillis;
    batteryVoltage = analogRead(BATT_VOLT) * VREF/1023 * 25/10;
    batteryChargeStatus = digitalRead(BATT_STAT);
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print("\tBattery Charge Status: ");
    Serial.println(batteryChargeStatus);
  }
  
}