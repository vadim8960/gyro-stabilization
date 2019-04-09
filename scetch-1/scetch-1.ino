#include "SparkFunMPU9250-DMP.h"

MPU9250_DMP mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (mpu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(12000);
    }
  }
  mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10);
  while (!mpu.fifoAvailable() && !mpu.dmpUpdateFifo() == INV_SUCCESS); 
}

void loop() {
  mpu.updateGyro();

  Serial.print(mpu.calcGyro(mpu.gx)); Serial.print(' ');
  Serial.print(mpu.calcGyro(mpu.gy)); Serial.print(' ');
  Serial.print(mpu.calcGyro(mpu.gz)); Serial.println();
}
