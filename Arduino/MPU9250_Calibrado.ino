#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  MPU9250Setting setting;

  setting.accel_fs_sel = ACCEL_FS_SEL::A8G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if (!mpu.setup(0x68, setting)) { 
    Serial.println("MPU9250 no encontrado. Revisa la conexión/I2C!");
    while (1);
  }

  mpu.verbose(true);
  delay(2000);

  Serial.println("CAlibrando Accel/Gyro... mantén el sensor quieto");
  mpu.calibrateAccelGyro();
  delay(1000);

  Serial.println("Calibrando Magnetómetro... mueve el sensor en forma de 8");
  mpu.calibrateMag();
  mpu.verbose(false);
  Serial.println("Calibración completa");

}

void loop() {
  if (mpu.update()){
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();

    float mx = mpu.getMagX();
    float my = mpu.getMagY();
    float mz = mpu.getMagZ();

    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.println(mz);
  }
  delay(20);

}


