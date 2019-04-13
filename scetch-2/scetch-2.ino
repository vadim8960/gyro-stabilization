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
}
