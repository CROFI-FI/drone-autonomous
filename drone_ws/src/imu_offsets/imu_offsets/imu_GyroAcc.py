import time
import yaml
from smbus2 import SMBus


# Direccion I2C del MPU6050
MPU_ADDR = 0x68

# Registros importantes
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


ACCEL_SCALE_FACTOR = 16384.0  # LSB/g for ±2g range
GYRO_SCALE_FACTOR = 131.0     # LSB/(°/s) for ±250°/s range

# Constantes fisicas
GRAVITY_G = 9.80665
PI = 3.14159265359

def read_word(bus, reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

def main():
    bus = None  
    try:
        bus = SMBus(1)
        
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        
        # Escribir 0 en ACCEL_CONFIG para +-2g
        bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0)
        # Escribir 0 en GYRO_CONFIG para +-250°/s
        bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0)

        print("Sensor MPU6050 inicializado.")
        print("Manten el sensor completamente quieto y nivelado sobre una superficie plana...")
        time.sleep(5)

        print("Iniciando calibracion... Tomando 500 muestras.")
        
        # Acumuladores para los valores crudos
        accel_raw_offsets = [0, 0, 0]
        gyro_raw_offsets = [0, 0, 0]
        samples = 500

        for _ in range(samples):
            accel_raw_offsets[0] += read_word(bus, ACCEL_XOUT_H)
            accel_raw_offsets[1] += read_word(bus, ACCEL_XOUT_H + 2)
            accel_raw_offsets[2] += read_word(bus, ACCEL_XOUT_H + 4)
            
            gyro_raw_offsets[0] += read_word(bus, GYRO_XOUT_H)
            gyro_raw_offsets[1] += read_word(bus, GYRO_XOUT_H + 2)
            gyro_raw_offsets[2] += read_word(bus, GYRO_XOUT_H + 4)
            
            time.sleep(0.004)

        # Calcula el promedio de los offsets crudos
        avg_accel_raw = [val / samples for val in accel_raw_offsets]
        avg_gyro_raw = [val / samples for val in gyro_raw_offsets]

        print("Promedios crudos calculados.")

        # --- Conversion a Unidades Fisicas ---
        gyro_offsets_rad_s = [val / GYRO_SCALE_FACTOR * (PI / 180) for val in avg_gyro_raw]
        
        # Para el acelerometro, el offset es la diferencia con el valor esperado.
        # X e Y deberian ser 0g. Z deberia ser 1g.
        accel_offsets_ms2 = [
            avg_accel_raw[0] / ACCEL_SCALE_FACTOR * GRAVITY_G,
            avg_accel_raw[1] / ACCEL_SCALE_FACTOR * GRAVITY_G,
            (avg_accel_raw[2] - ACCEL_SCALE_FACTOR) / ACCEL_SCALE_FACTOR * GRAVITY_G
        ]

        print("\n--- Offsets de Calibracion Calculados ---")
        print(f"Acelerometro (m/s^2): {accel_offsets_ms2}")
        print(f"Giroscopio (rad/s):   {gyro_offsets_rad_s}")

        # Guardar en archivo YAML
        calibration_data = {
            "accel_offsets": accel_offsets_ms2,
            "gyro_offsets": gyro_offsets_rad_s
        }
        with open("imu_calibration.yaml", "w") as f:
            yaml.dump(calibration_data, f, indent=4)

    except OSError as e:
        print(f"Error de comunicacion I2C: {e}")
        print("Verifica la conexion del sensor MPU6050 y la direccion I2C (0x{MPU_ADDR:X}).")
    
    finally:
        if bus:
            bus.close()
            print("Conexion I2C cerrada.")

if __name__ == "__main__":
    main()