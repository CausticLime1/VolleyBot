#include <Wire.h>

float rollRate, pitchRate, yawRate;
float rollCal, pitchCal, yawCal;
float accX, accY, accZ;
float roll, pitch;

const int motor11APin = PA0;
const int motor12APin = PA1;

//Motor 2 Pins

const int motor21APin = PA3;
const int motor22APin = PA2;

//Motor 3 Pins

const int motor31APin = PA6;
const int motor32APin = PA7;

//Motor Control

float motor1Write = 0;
float motor2Write = 0;
float motor3Write = 0;

//PID Control

float rollError;
float pitchError;
float rollIntegral = 0;
float pitchIntegral = 0;
float rollDerivative = 0;
float pitchDerivative = 0;
float prevRoll = 0;
float prevPitch = 0;
float PIDRoll = 0;
float PIDPitch = 0;

//PID Tuning

float gainP = 1;
float gainI = 1;
float gainD = 1;

//Variables for the kalman filter
float kalmanAngleRoll = 0, kalmanUncertaintyAngleRoll = 2*2;
float kalmanAnglePitch = 0, kalmanUncertaintyAnglePitch = 2*2;
float kalman1DOutput[] = {0, 0};

uint32_t loopTimer; 


void gyroSignals(void){

  //Configure Gyroscope Scale
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //Configure Low-Pass Filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  //Configure Acceleration Scale
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Reading Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t accXLSB = Wire.read() << 8 | Wire.read();
  int16_t accYLSB = Wire.read() << 8 | Wire.read();
  int16_t accZLSB = Wire.read() << 8 | Wire.read();

  accX = (accXLSB / 4096.0) - 0.05;
  accY = (accYLSB / 4096.0);
  accZ = (accZLSB / 4096.0) -0.05;

  //Reading Gyroscope 
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  rollRate = (float)gyroX / 65.5;
  pitchRate = (float)gyroY / 65.5;
  yawRate = (float)gyroZ / 65.5;
  
  roll = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * (180.0/M_PI);
  pitch = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * (180.0/M_PI);
}

void calibrateGyro() {
  int32_t rollSum = 0, pitchSum = 0, yawSum = 0;
  for (int i = 0; i < 500; i++) {
    gyroSignals();
    rollSum += rollRate;
    pitchSum += pitchRate;
    yawSum += yawRate;
    delay(5);
  }
  rollCal = rollSum / 500.0f;
  pitchCal = pitchSum / 500.0f;
  yawCal = yawSum / 500.0f;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PB12, OUTPUT);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  calibrateGyro();
}

void analogTrimmer(float* input){
  if (abs(*input) > 255){
    if (*input > 0){
      *input = 255;
    }
    else{
      *input = -255;
    }
  }
}

void kalman_1d(float* KalmanState, float* KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  *KalmanState = *KalmanState + 0.004 * KalmanInput;
  *KalmanUncertainty = *KalmanUncertainty + 0.004 * 0.004 * 4.0 * 4.0;
  float KalmanGain = *KalmanUncertainty * (1/ ((*KalmanUncertainty) + 3.0*3.0));
  *KalmanState = *KalmanState + KalmanGain * (KalmanMeasurement - *KalmanState);
  *KalmanUncertainty = (1.0 - KalmanGain) * (*KalmanUncertainty);
  kalman1DOutput[0] = *KalmanState;
  kalman1DOutput[1] = *KalmanUncertainty;
}

void loop() {
  gyroSignals();
  rollRate -= rollCal;
  pitchRate -= pitchCal;
  yawRate -= yawCal;

  //Filter of Roll Degrees
  kalman_1d(&kalmanAngleRoll, &kalmanUncertaintyAngleRoll, rollRate, roll);
  kalmanAngleRoll = kalman1DOutput[0];
  kalmanUncertaintyAngleRoll = kalman1DOutput[1];

  //Filter of Pitch Degrees
  kalman_1d(&kalmanAnglePitch, &kalmanUncertaintyAnglePitch, pitchRate, pitch);
  kalmanAnglePitch = kalman1DOutput[0];
  kalmanUncertaintyAnglePitch = kalman1DOutput[1];

  rollError = rollCal - kalmanAngleRoll;
  pitchError = pitchCal - kalmanAnglePitch;

  rollIntegral += rollError * 0.004;
  pitchIntegral += pitchError * 0.004;

  rollDerivative = (kalmanAngleRoll - prevRoll)/0.004;
  pitchDerivative = (kalmanAnglePitch - prevPitch)/0.004;

  PIDRoll = gainP * rollError + gainI * rollIntegral + gainD * rollDerivative;
  PIDPitch = gainP * pitchError + gainI * pitchIntegral + gainD * pitchDerivative;

  motor1Write = (-0.5 * PIDPitch) + (0.866 * PIDRoll); 
  motor2Write = PIDRoll;
  motor3Write = (-0.5 * PIDPitch) - (0.866 * PIDRoll);

  if (motor1Write < 0){
    motor1Write -= 64;
    analogTrimmer(&motor1Write);
    analogWrite(motor11APin, abs(motor1Write));
    digitalWrite(motor12APin, 0);
  }
  else if (motor1Write > 0){
    motor1Write += 64;
    analogTrimmer(&motor1Write);
    digitalWrite(motor11APin, 0);
    analogWrite(motor12APin, abs(motor1Write));
  }

  if (motor2Write < 0){
    motor2Write -= 64;
    analogTrimmer(&motor2Write);
    analogWrite(motor21APin, abs(motor2Write));
    digitalWrite(motor22APin, 0);
  }
  else if (motor2Write > 0){
    motor2Write += 64;
    analogTrimmer(&motor2Write);
    digitalWrite(motor21APin, 0);
    analogWrite(motor22APin, abs(motor2Write));
  }

  if (motor3Write < 0){
    motor3Write -= 64;
    analogTrimmer(&motor3Write);
    analogWrite(motor31APin, abs(motor3Write));
    digitalWrite(motor32APin, 0);
  }
  else if (motor3Write > 0){
    motor3Write += 64;
    analogTrimmer(&motor3Write);
    digitalWrite(motor31APin, 0);
    analogWrite(motor32APin, abs(motor3Write));
  }

  prevRoll = kalmanAngleRoll;
  prevPitch = kalmanAnglePitch;

  Serial.print("Roll:");
  Serial.print(kalmanAngleRoll);
  Serial.print("\tPitch:");
  Serial.print(kalmanAnglePitch);
  Serial.print("\tMotor 1:");
  Serial.print(motor1Write);
  Serial.print("\tMotor 2:");
  Serial.print(motor2Write);
  Serial.print("\tMotor 3:");
  Serial.println(motor3Write);

  while (micros() - loopTimer < 4000);
  loopTimer = micros();
}
