import smbus2
from smbus2 import SMBus, i2c_msg
import time

# MPU6500 I2C address
MPU6500_ADDR = 0x68
ARDUINO_ADDR = 0x08

# MPU6500 Register addresses
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize I2C bus
bus = smbus2.SMBus(1)  # 1 for Raspberry Pi, use 0 for older versions

with open('data.txt', 'w') as f:
    pass

def send(content):
    with SMBus(7) as motorBus:
        # Create an I2C message with the data
        msg = i2c_msg.write(ARDUINO_ADDR, content)
        
        # Write the message to the I2C bus
        motorBus.i2c_rdwr(msg)
        
    print("Data sent successfully")

def initialize_mpu6500():
    # Wake up the MPU6500 as it starts in sleep mode
    bus.write_byte_data(MPU6500_ADDR, PWR_MGMT_1, 0)
    time.sleep(5)
    

def read_raw_data(reg_addr):
    # Read two bytes of data from the specified register
    high = bus.read_byte_data(MPU6500_ADDR, reg_addr)
    low = bus.read_byte_data(MPU6500_ADDR, reg_addr + 1)
    value = ((high << 8) | low) & 0xFFFF
    if value > 32767:
        value -= 65536
    return value

def get_accel_data():
    # Read accelerometer data
    ax = read_raw_data(ACCEL_XOUT_H)
    ay = read_raw_data(ACCEL_XOUT_H + 2)
    az = read_raw_data(ACCEL_XOUT_H + 4)
    return ax, ay, az

def get_gyro_data():
    # Read gyroscope data
    gx = read_raw_data(GYRO_XOUT_H)
    gy = read_raw_data(GYRO_XOUT_H + 2)
    gz = read_raw_data(GYRO_XOUT_H + 4)
    return gx, gy, gz

initialize_mpu6500()

# Low-pass filter function
def low_pass_filter(new_value, previous_value, alpha=0.5):
    # alpha is the smoothing factor (between 0 and 1)
    return alpha * new_value + (1 - alpha) * previous_value

# Example usage:
previous_ax, previous_ay, previous_az = 0, 0, 0
previous_gx, previous_gy, previous_gz = 0, 0, 0

for i in range(100):
    ax, ay, az = get_accel_data()
    gx, gy, gz = get_gyro_data()

    # Apply low-pass filter
    ax = low_pass_filter(ax, previous_ax)
    ay = low_pass_filter(ay, previous_ay)
    az = low_pass_filter(az, previous_az)

    gx = low_pass_filter(gx, previous_gx)
    gy = low_pass_filter(gy, previous_gy)
    gz = low_pass_filter(gz, previous_gz)

    # Update previous values for the next iteration
    previous_ax, previous_ay, previous_az = ax, ay, az
    previous_gx, previous_gy, previous_gz = gx, gy, gz

    with open('data.txt', 'a') as f:
        f.write(f'{ax},{ay},{az},{gx},{gy},{gz}\n')
send("m1-100m2-100m3-100m4-100")
for i in range(500):
    ax, ay, az = get_accel_data()
    gx, gy, gz = get_gyro_data()

    # Apply low-pass filter
    ax = low_pass_filter(ax, previous_ax)
    ay = low_pass_filter(ay, previous_ay)
    az = low_pass_filter(az, previous_az)

    gx = low_pass_filter(gx, previous_gx)
    gy = low_pass_filter(gy, previous_gy)
    gz = low_pass_filter(gz, previous_gz)

    # Update previous values for the next iteration
    previous_ax, previous_ay, previous_az = ax, ay, az
    previous_gx, previous_gy, previous_gz = gx, gy, gz

    with open('data.txt', 'a') as f:
        f.write(f'{ax},{ay},{az},{gx},{gy},{gz},{time.time()}\n')
send("m1-000m2-000m3-000m4-000")
