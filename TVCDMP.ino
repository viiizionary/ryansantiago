/*
   Description: Teensy Model Rocket Gimbal Controller
   Made for Teensy board (e.g., 4.0/4.1)
   Author: Ryan Santiago but Adapted 
   Pinout:
     - MPU6050 INT: pin 2 
     - Servo X: pin 9
     - Servo Y: pin 10
*/

#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#define LED_PIN LED_BUILTIN
#define MPU_INT_PIN 2

Servo ServoX;
Servo ServoY;
bool allowCalibration = true;

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
float baseAltitude = 0;

MPU6050 mpu;
Adafruit_BMP085 bmp;

uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

#define PITCH 1
#define ROLL 2
#define YAW 0

double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

double KpX = 2, KiX = 0.005, KdX = 1;
double KpY = 2, KiY = 0.005, KdY = 1;
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

int buffersize = 200;
int acel_deadzone = 8;
int giro_deadzone = 1;

int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void setup() {
  ServoX.attach(9);
  ServoY.attach(10);
  ServoX.write(90);
  ServoY.write(90);
  delay(500);

  Wire.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("Initializing BMP180..."));
  if (!bmp.begin()) {
    Serial.println(F("BMP180 init failed! Check wiring."));
    while (1);
  }

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  ax_offset = 1118; ay_offset = 513; az_offset = 1289;
  gx_offset = 64; gy_offset = -1; gz_offset = -33;

  pinMode(LED_PIN, OUTPUT);

  calibrate();
  initialize();
}

void initialize() {
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  myPIDX.SetOutputLimits(-30, 30);
  myPIDY.SetOutputLimits(-30, 30);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  SetpointX = 0;
  SetpointY = 0;

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }
}

void loop() {
  MainMenu();
}

void myloop() {
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) { mpu.resetFIFO(); return; }
  if (fifoCount < packetSize) return;

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.resetFIFO();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  mpuRoll  = ypr[PITCH] * 180 / M_PI;
  mpuPitch = ypr[ROLL] * 180 / M_PI;
  mpuYaw   = ypr[YAW] * 180 / M_PI;

  InputX = mpuPitch;
  InputY = mpuRoll;
  myPIDX.Compute();
  myPIDY.Compute();

  ServoX.write(constrain(-OutputX + 90, 60, 120));
  ServoY.write(constrain(OutputY + 90, 60, 120));

  float currentAlt = bmp.readAltitude();
  float relAlt = currentAlt - baseAltitude;

  Serial.print("Pitch: "); Serial.print(mpuPitch, 2);
  Serial.print(" | Roll: "); Serial.print(mpuRoll, 2);
  Serial.print(" | Altitude: "); Serial.print(relAlt, 2);
  Serial.print(" m | ServoX: "); Serial.print(ServoX.read());
  Serial.print(" | ServoY: "); Serial.println(ServoY.read());

  delay(10);
}

void calibrate() {
  Serial.println("\nMPU6050 Calibration");
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
      meansensors(); state++; delay(100);
    } else if (state == 1) {
      calibration(); state++; delay(100);
    } else if (state == 2) {
      meansensors();
      baseAltitude = bmp.readAltitude(); // BMP180 zero reference
      Serial.println("Calibration complete\n");
      break;
    }
  }
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) {
      buff_ax += ax; buff_ay += ay; buff_az += az;
      buff_gx += gx; buff_gy += gy; buff_gz += gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++; delay(2);
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    meansensors();
    if (abs(mean_ax) <= acel_deadzone) ready++; else ax_offset -= mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++; else ay_offset -= mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++; else az_offset += (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= giro_deadzone) ready++; else gx_offset -= mean_gx / (giro_deadzone + 1);
    if (abs(mean_gy) <= giro_deadzone) ready++; else gy_offset -= mean_gy / (giro_deadzone + 1);
    if (abs(mean_gz) <= giro_deadzone) ready++; else gz_offset -= mean_gz / (giro_deadzone + 1);
    if (ready == 6) break;
  }
}

void MainMenu() {
  char commandbuffer[200]; int i = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != ';' && c != '\n') commandbuffer[i++] = c;
    else { commandbuffer[i] = '\0'; break; }
  }
  if (commandbuffer[0] != '\0') {
    if (strncmp(commandbuffer, "calibration", 11) == 0 && allowCalibration) {
      state = 0;
      calibrate();
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);
      mpu.setXGyroOffset(gx_offset);
      mpu.setYGyroOffset(gy_offset);
      mpu.setZGyroOffset(gz_offset);
      allowCalibration = false;
    }
  }
  myloop();
}
