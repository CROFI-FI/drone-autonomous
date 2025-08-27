#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  Serial.println("MPU9250 listo");
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  float gx = mySensor.gyroX();
  float gy = mySensor.gyroY();
  float gz = mySensor.gyroZ();hh

  float mx = mySensor.magX();
  float my = mySensor.magY();
  float mz = mySensor.magZ();

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.println(mz); 

  delay(50);
}