#include "Motor.h"
#include <Arduino_LSM9DS1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Motor + Encoder Pins
#define PWMA 6
#define in1 8
#define in2 7
#define C1A 3
#define C2A 5

#define MOTOR_STANDBY 9

#define PWMB 11
#define in3 10
#define in4 12
#define C1B 2
#define C2B 4

// MPU6050 Variables
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float ypr[3];

// High-Level Orientation Control
const float kp_theta = 2;

// Low-Level Speed Control Parameters
const int maxRPM = 200;
const float CPR = 12;
const float gearRatio = 100.37;
const float conversionRatio = 4./CPR*60/gearRatio;
const int loopInterval = 50;
unsigned long lastLoop;
int target[2];
float v[2];

Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);

// Debugging / Modes
const bool print = false;
const bool logIMU = false;
const bool serialConnection = true;

// Complimentary Filter
float a[3], r[3], m[3];
const float alpha = .95; // 0-1
float orientation[3];
float thetaM, phiM;
float dt;
unsigned long millisOld;

void setup() {
  Serial.begin(9600);
  //Serial1.begin(9600);

  IMUsetup();

  attachInterrupt(digitalPinToInterrupt(C1A), motorAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(C1B), motorBInterrupt, RISING);

  digitalWrite(MOTOR_STANDBY, HIGH);

  motorA.tunePID(1, 5, 0);
  motorB.tunePID(1, 5, 0);
}

void loop() {
  controlLoop();
}

void controlLoop() {
  if(millis()-lastLoop < loopInterval) return;
  lastLoop = millis();
  readIMU();

  // Read serial
  if(serialConnection) {
    if (Serial.available()) {
      readIntArray(target, 2);
      Serial.readStringUntil('\n');
    }
  }
  else {
    target[0] = maxRPM * round(sin(0.3 * micros() / 1e6));
    target[1] = maxRPM * round(cos(0.3 * micros() / 1e6));
  }

  // Run orientation PID, ensure rpmIn does not exceed maximum
  float rpmIn = orientationControl(target[0]);
  if(abs(rpmIn) > maxRPM) rpmIn = rpmIn/abs(rpmIn) * maxRPM;

  v[0] = controlMotor(target[0], &motorA);
  v[1] = controlMotor(target[1], &motorB);

  if(serialConnection) {
    printArray(orientation,3);
    printArray(v,2);
    Serial.println();
  }

  // Print to serial for debugging
  if (print) debugger();
}

float orientationControl(float target) {
  float e = target - orientation[0];

  // PID Placeholder

  return kp_theta*e;
}

// Helper function to read and filter IMU data
void readIMU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    orientation[0] = ypr[0] * 180/M_PI;
    orientation[1] = ypr[1] * 180/M_PI;
    orientation[2] = ypr[2] * 180/M_PI;
  }

  /*
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(a[0], a[1], a[2]);
  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(r[0], r[1], r[2]);
  if(IMU.magneticFieldAvailable())
    IMU.readMagneticField(m[0], m[1], m[2]);

  // Complimentary filter
  dt = (millis()-millisOld)/1000.;
  millisOld = millis();

  thetaM = atan2(a[0],sqrt(a[1]*a[1]+a[2]*a[2]))/PI*180;
  phiM = atan2(m[1], m[2])/PI*180;//atan2(a[1],sqrt(a[0]*a[0]+a[2]*a[2]))/PI*180;

  orientation[0] = alpha*(orientation[0] + dt*r[1]) + (1-alpha)*thetaM;
  orientation[1] = alpha*(orientation[1] - dt*r[0]) + (1-alpha)*phiM;

  float theta = orientation[0]/180 * PI;
  float phi = orientation[1]/180 * PI;

  float psi = 0;//atan2(m[1], m[2]);//atan2(m[2]*sin(phi) + m[1]*cos(phi), m[0]*cos(theta) + sin(theta)*(-m[1]*sin(phi) + m[2]*cos(phi)));

  orientation[2] = psi/PI * 180;
  */
}

float controlMotor(float vt, Motor *mtr) {
  float v = mtr->updateVelocity();
  float u = mtr->rpmControl(vt);
  mtr->drive(u);
  return v;
}

// Print stuff
void debugger() {
  if(logIMU) { // Print IMU data
    Serial.print(String(dt) + ' ');
    //Serial.print(String(thetaM) + ' ' + String(phiM) + ' ');
    Serial.print(String(orientation[0]) + ' ' + String(orientation[1]) + ' ' + String(orientation[2]) + ' ');
    Serial.print("180 -180");
  }
  else { // Print Motor RPM data
    Serial.print(String(target[0]) + ' ' + String(v[0]) + ' ');
    Serial.print(String(target[1]) + ' ' + String(v[1]) + ' ');
    Serial.print(String(maxRPM) + " -" + String(maxRPM));
  }
  Serial.println();
}

// Motor Interrupts from Encoders
void motorAInterrupt() {
  motorA.readEncoder();
}
void motorBInterrupt() {
  motorB.readEncoder();
}

void readIntArray(int *arr, int length) {
  for(int i = 0; i<length; i++)
    arr[i] = Serial.readStringUntil(',').toInt();
}

void printArray(float *arr, int length) {
  for(int i=0; i<length; i++)
    Serial.print(String(arr[i]) + ',');
}

void IMUsetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      Wire.setWireTimeout(3000, true);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}