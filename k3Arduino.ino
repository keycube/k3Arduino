#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdlib.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE 
// implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/*
 * MPU6050
 */

// Actual quaternion components in a [w, x, y, z] format.
// (not best for parsing on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// Euler angles (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock.
// (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// Yaw/Pitch/Roll angles (in degrees) calculated from the quaternions coming from the FIFO. 
// Note this also requires gravity vector calculations and suffer from gimbal lock. 
// (for more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// Acceleration components with gravity removed. This acceleration reference frame is not 
// compensated for orientation, so +X is always +X according to the sensor. 
//#define OUTPUT_READABLE_REALACCEL

// Acceleration components with gravity removed and adjusted for the world frame of reference.
// (Yaw is relative to initial orientation, since no magnetometer is present in this case.)
//#define OUTPUT_READABLE_WORLDACCEL

// Output that matches the format used for the InvenSense teapot demo.
//#define OUTPUT_TEAPOT

MPU6050 mpu;
#define INTERRUPT_PIN 2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Interrupt Detection Routine
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

char cstr[16];

/*
 * Bluetooth
 */

SoftwareSerial bluetoothSerial(10, 11); // RX, TX

/*
 * Matrix 64
 */
 
byte outputs64[] = {42, 46, 43, 47, 40, 44, 41, 45};
const int outputs64Count = 8;
 
byte inputs64[] = {34, 38, 35, 39, 32, 36, 33, 37};
const int inputs64Count = 8;

byte keys64State[outputs64Count][inputs64Count];
const char* keys64Value[outputs64Count][inputs64Count];

/*
 * Matrix 16
 */
 
byte outputs16[] = {28, 29, 30, 31};
const int outputs16Count = 4;

byte inputs16[] = {24, 25, 26, 27};
const int inputs16Count = 4;

byte keys16State[outputs16Count][inputs16Count];
const char* keys16Value[outputs16Count][inputs16Count];


void setup() {

  // Start serial connection if connected through Bluetooth
  bluetoothSerial.begin(9600);
  
  // Start Serial connection if connected through USB
  Serial.begin(19200);

  // R, G, U (blue), Y, W
  // 0123456789ABCDEF

  /*
   * Matrix 64
   */
   
  // red
  keys64Value[0][0] = "+r6";
  keys64Value[0][1] = "+rE";
  keys64Value[0][2] = "+r2";
  keys64Value[0][3] = "+rA";
  keys64Value[1][0] = "+r4";
  keys64Value[1][1] = "+rC";
  keys64Value[1][2] = "+r0";
  keys64Value[1][3] = "+r8";
  keys64Value[2][0] = "+r7";
  keys64Value[2][1] = "+rF";
  keys64Value[2][2] = "+r3";
  keys64Value[2][3] = "+rB";
  keys64Value[3][0] = "+r5";
  keys64Value[3][1] = "+rD";
  keys64Value[3][2] = "+r1";
  keys64Value[3][3] = "+r9";

  // green
  keys64Value[4][0] = "+g6";
  keys64Value[4][1] = "+gE";
  keys64Value[4][2] = "+g2";
  keys64Value[4][3] = "+gA";
  keys64Value[5][0] = "+g4";
  keys64Value[5][1] = "+gC";
  keys64Value[5][2] = "+g0";
  keys64Value[5][3] = "+g8";
  keys64Value[6][0] = "+g7";
  keys64Value[6][1] = "+gF";
  keys64Value[6][2] = "+g3";
  keys64Value[6][3] = "+gB";
  keys64Value[7][0] = "+g5";
  keys64Value[7][1] = "+gD";
  keys64Value[7][2] = "+g1";
  keys64Value[7][3] = "+g9";

  // blue (u)
  keys64Value[0][4] = "+u6";
  keys64Value[0][5] = "+uE";
  keys64Value[0][6] = "+u2";
  keys64Value[0][7] = "+uA";
  keys64Value[1][4] = "+u4";
  keys64Value[1][5] = "+uC";
  keys64Value[1][6] = "+u0";
  keys64Value[1][7] = "+u8";
  keys64Value[2][4] = "+u7";
  keys64Value[2][5] = "+uF";
  keys64Value[2][6] = "+u3";
  keys64Value[2][7] = "+uB";
  keys64Value[3][4] = "+u5";
  keys64Value[3][5] = "+uD";
  keys64Value[3][6] = "+u1";
  keys64Value[3][7] = "+u9";

  // yellow
  keys64Value[4][4] = "+y5";
  keys64Value[4][5] = "+y7";
  keys64Value[4][6] = "+y4";
  keys64Value[4][7] = "+y6";
  keys64Value[5][4] = "+yD";
  keys64Value[5][5] = "+yF";
  keys64Value[5][6] = "+yC";
  keys64Value[5][7] = "+yE";
  keys64Value[6][4] = "+y1";
  keys64Value[6][5] = "+y3";
  keys64Value[6][6] = "+y0";
  keys64Value[6][7] = "+y2";
  keys64Value[7][4] = "+y9";
  keys64Value[7][5] = "+yB";
  keys64Value[7][6] = "+y8";
  keys64Value[7][7] = "+yA";

  for (int i = 0; i < outputs64Count; i++) {
    pinMode(outputs64[i], OUTPUT);
    digitalWrite(outputs64[i], HIGH);
  }
  
  for (int i = 0; i < inputs64Count; i++) {
    pinMode(inputs64[i], INPUT_PULLUP);
    digitalWrite(inputs64[i], HIGH);
  }

  for (int i = 0; i < outputs64Count; i++) {
    for (int j = 0; j < inputs64Count; j++) {
      keys64State[i][j] = 1;
    }
  }

  /* 
   * Matrix 16 
   */
  
  // white (matrix 16)
  keys16Value[0][0] = "+wF";
  keys16Value[0][1] = "+wE";
  keys16Value[0][2] = "+wD";
  keys16Value[0][3] = "+wC";
  keys16Value[1][0] = "+wB";
  keys16Value[1][1] = "+wA";
  keys16Value[1][2] = "+w9";
  keys16Value[1][3] = "+w8";
  keys16Value[2][0] = "+w7";
  keys16Value[2][1] = "+w6";
  keys16Value[2][2] = "+w5";
  keys16Value[2][3] = "+w4";
  keys16Value[3][0] = "+w3";
  keys16Value[3][1] = "+w2";
  keys16Value[3][2] = "+w1";
  keys16Value[3][3] = "+w0";

  for (int i = 0; i < outputs16Count; i++) {
    pinMode(outputs16[i], OUTPUT);
    digitalWrite(outputs16[i], HIGH);
  }
  
  for (int i = 0; i < inputs16Count; i++) {
    pinMode(inputs16[i], INPUT_PULLUP);
    digitalWrite(inputs16[i], HIGH);
  }

  for (int i = 0; i < outputs16Count; i++) {
    for (int j = 0; j < inputs16Count; j++) {
      keys16State[i][j] = 1;
    }
  }

  /*
   * MPU6050
   */

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // TODO: Set appropriate gyro/accel offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void readPrintMatrix64() {
  for (int i = 0; i < outputs64Count; i++) {
    digitalWrite(outputs64[i], LOW);
    for (int j = 0; j < inputs64Count; j++) {
      int value = digitalRead(inputs64[j]);
      if (value != keys64State[i][j]) {
        bluetoothSerial.write(keys64Value[i][j]);
        Serial.write(keys64Value[i][j]);
      }
      keys64State[i][j] = value;
    }
    digitalWrite(outputs64[i], HIGH);
  }
}

void readPrintMatrix16() {
  for (int i = 0; i < outputs16Count; i++) {
    digitalWrite(outputs16[i], LOW);
    for (int j = 0; j < inputs16Count; j++) {
      int value = digitalRead(inputs16[j]);
      if (value != keys16State[i][j]) {
        bluetoothSerial.write(keys16Value[i][j]);
        Serial.write(keys16Value[i][j]);
      }
      keys16State[i][j] = value;
    }
    digitalWrite(outputs16[i], HIGH);
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    readPrintMatrix64();
    readPrintMatrix16();    
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // TODO: here chose gyro/accel format
    #ifndef OUTPUT_TEAPOT
      Serial.write("*");
      mpu.dmpGetQuaternion(&q, fifoBuffer);
    #endif
    
    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values (multiplied by 100) in easy matrix form: x y z w
      // (divide them by 100 on remote host)
      Serial.write(":");
      Serial.write(itoa(q.w*100, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(q.x*100, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(q.y*100, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(q.z*100, cstr, 10));
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetEuler(euler, &q);
      Serial.write(":");
      Serial.write(itoa(euler[0] * 180/M_PI, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(euler[1] * 180/M_PI, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(euler[2] * 180/M_PI, cstr, 10));
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.write(":");
      Serial.write(itoa(ypr[0] * 180/M_PI, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(ypr[1] * 180/M_PI, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(ypr[2] * 180/M_PI, cstr, 10));
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.write(":");
      Serial.write(itoa(aaReal.x, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(aaReal.y, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(aaReal.z, cstr, 10));
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.write(":");
      Serial.write(itoa(aaWorld.x, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(aaWorld.y, cstr, 10));
      Serial.write(":");
      Serial.write(itoa(aaWorld.z, cstr, 10));
    #endif

    #ifndef OUTPUT_TEAPOT
      Serial.write(".");
    #else
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    #endif
  }
}
