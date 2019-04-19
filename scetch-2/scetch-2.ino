#include "SparkFunMPU9250-DMP.h"

#include "AX12A.h"

#define k 0.01

double change = 0;
unsigned long timetook = 0;
unsigned long lasttook = 0;
double angles[3] = {0, 0, 0};
double zero_position_gyro[3] = {0, 0, 0};

MPU9250_DMP mpu;

void setup() {
  Serial.begin(9600);

  ax12a.begin(1000000, 2, &Serial1);
  ax12a.setEndless(1, false);
  ax12a.moveSpeed(1, 512, 100);
  delay(3000);
  ax12a.setEndless(1, true);

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

  for (unsigned iter = 0; iter < 100; ++iter) {
    mpu.updateFifo();
    zero_position_gyro[0] += mpu.calcGyro(mpu.gx);
    zero_position_gyro[1] += mpu.calcGyro(mpu.gy);
    zero_position_gyro[2] += mpu.calcGyro(mpu.gz);
  }

  zero_position_gyro[0] /= 100.0;
  zero_position_gyro[1] /= 100.0;
  zero_position_gyro[2] /= 100.0;

  Serial.println("End setup");
  delay(3000);
}

double clamp(double v, double v_min, double v_max) {
  if (v > v_max)
    return v_max;
  if (v < v_min)
    return v_min;
  return v;
}

double my_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
  mpu.updateGyro();
  mpu.updateAccel();

  double acc[3] = {0, 0, 0};
  acc[0] = mpu.calcAccel(mpu.ax);
  acc[1] = mpu.calcAccel(mpu.ay);
  acc[2] = mpu.calcAccel(mpu.az);
  double gyr[3] = {0, 0, 0};
  gyr[0] = mpu.calcGyro(mpu.gx);
  gyr[1] = mpu.calcGyro(mpu.gy);
  gyr[2] = mpu.calcGyro(mpu.gz);

  lasttook = timetook;
  timetook = millis();

  double change[3] = {0, 0, 0};
  double dt = (timetook - lasttook) / 1000.0;


  for (unsigned iter = 0; iter < 3; ++iter) {
    if (fabs(gyr[iter] - zero_position_gyro[iter]) > 5.0) 
      gyr[iter] = gyr[iter] - zero_position_gyro[iter];
  }

  angles[0] = (1 - k) * (angles[0] + gyr[0] * dt) + acc[0] * k;
  angles[1] = (1 - k) * (angles[1] + gyr[1] * dt) + acc[1] * k;
  angles[2] = (1 - k) * (angles[2] + gyr[2] * dt) + acc[2] * k;

  double norm = 0;
  for (unsigned iter = 0; iter < 3; ++iter)
    norm += (angles[iter] * angles[iter]);
  norm = sqrt(norm);

  for (unsigned iter = 0; iter < 3; ++iter) {
    angles[iter] /= norm;
  }

  for (unsigned iter = 0; iter < 3; ++iter) {
    Serial.print(angles[iter]); Serial.print(' ');
  }
  Serial.println();

  double v = clamp(angles[1], -1.0, -0.22);
  
  v = my_map(v, -1.0, -0.22, -1.0, 1.0);
  // Serial.println(v);

  double error = v;
  ax12a.turn(1, error > 0, (int) (fabs(error) * 125));

  delay(10);
}
 
  