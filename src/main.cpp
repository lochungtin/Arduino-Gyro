#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

// packet
float packet[3];

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();

        mpu.setXAccelOffset(-902);
        mpu.setYAccelOffset(-475);
        mpu.setZAccelOffset(1643);
        mpu.setXGyroOffset(-7);
        mpu.setYGyroOffset(777);
        mpu.setZGyroOffset(126);
    }
}

void loop() {
    if (devStatus != 0) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          mpu.dmpGetQuaternion(&q, fifoBuffer);

          char buffer[18];
          int w = q.w * 100;
          int x = q.x * 100;
          int y = q.y * 100;
          int z = q.z * 100;

          sprintf(buffer, "H%04d%04d%04d%04dT", w, x, y, z);
          Serial.println(buffer);
      }
}
