#include "SparkFunMPU9250-DMP.h"

#include "AX12A.h"

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
  mpu.dmpEnable3Quat();
  while (!mpu.fifoAvailable() && !mpu.dmpUpdateFifo() == INV_SUCCESS); 

  // ax12a.begin(1000000, 2, &Serial1);
}

int change = 0;
double position = 0;
unsigned long timetook = 0;
unsigned long lasttook = 0;

void loop() {
  // mpu.dmpUpdateFifo();
  // float quat[4] = {};
  // quat[0] = mpu.calcQuat(mpu.qw);
  // quat[1] = mpu.calcQuat(mpu.qx);
  // quat[2] = mpu.calcQuat(mpu.qy);
  // quat[3] = mpu.calcQuat(mpu.qz);

  // for (unsigned iter = 0; iter < 4; ++iter) {
  //   Serial.print(quat[iter]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  mpu.updateGyro();
  mpu.updateAccel();
  double gy = mpu.calcGyro(mpu.gy);
  double ay = mpu.calcAccel(mpu.ay);
  if (fabs(gy + 1) > 3)
    change = gy + 1;

  lasttook = timetook;
  timetook = millis();

  double dt = (double(timetook) - double(lasttook)) / 1000.0;
  double degch = change * dt;

  position += degch;
  
  Serial.println(position);
  change = 0;
  delay(10);

//   ax12a.move(1, 400);
//   delay(10);
//   ax12a.move(1, 600);
//   delay(10);
//
  // mpu.updateAccel();
  // Serial.print(mpu.calcAccel(mpu.ax)); Serial.print(' ');
  // Serial.print(mpu.calcAccel(mpu.ay)); Serial.print(' ');
  // Serial.print(mpu.calcAccel(mpu.az)); Serial.println();
}
