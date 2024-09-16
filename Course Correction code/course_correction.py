import smbus2
from smbus2 import SMBus, i2c_msg
import time

# MPU6500 Registers
MPU6500_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_ZOUT_H = 0x47

# Initialize the I2C bus
bus = smbus2.SMBus(1)


def send(message):
    with SMBus(7) as bus:
            # Create an I2C message with the data
            msg = i2c_msg.write(8, message)
            # Write the message to the I2C bus
            bus.i2c_rdwr(msg)

def read_word_2c(addr):
    high = bus.read_byte_data(MPU6500_ADDR, addr)
    low = bus.read_byte_data(MPU6500_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def get_accel_data():
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0  # Scale factor for accelerometer
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    return accel_x, accel_y, accel_z

def get_gyro_data():
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0  # Scale factor for gyroscope
    return gyro_z

def calibrate_imu(samples=1000):
    accel_bias_x, accel_bias_y, accel_bias_z = 0, 0, 0
    gyro_bias_z = 0
    
    print("Calibrating IMU...")
    send('m1-100m2-100m3-100m4-100')
    for _ in range(samples):
        ax, ay, az = get_accel_data()
        gz = get_gyro_data()
        
        accel_bias_x += ax
        accel_bias_y += ay
        accel_bias_z += az - 1.0  # Subtracting 1g from Z-axis to account for gravity
        gyro_bias_z += gz
        
        time.sleep(0.002)  # Small delay to avoid overwhelming the I2C bus

    # Calculate average bias
    accel_bias_x /= samples
    accel_bias_y /= samples
    accel_bias_z /= samples
    gyro_bias_z /= samples
    
    print(f"Accel Biases -> X: {accel_bias_x:.3f}, Y: {accel_bias_y:.3f}, Z: {accel_bias_z:.3f}")
    print(f"Gyro Bias Z: {gyro_bias_z:.3f}")
    
    return accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_z

# Wake up the MPU6500
bus.write_byte_data(MPU6500_ADDR, 0x6B, 0)  # Reset the power management register to wake up the MPU6500

# Calibrate the IMU
accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_z = calibrate_imu()

# Initialize velocity components
vx, vy, vz = 0, 0, 0
last_time = time.time()

with open('data.txt', 'w') as f:
    pass

send('m1-100m2+100m3+100m4-100')
for i in range(500):
    # Calculate time difference
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Read accelerometer data
    ax, ay, az = get_accel_data()
    
    # Subtract the bias to get the corrected accelerometer readings
    ax -= accel_bias_x
    ay -= accel_bias_y
    az -= accel_bias_z

    # Read gyroscope data (only Z-axis)
    gz = get_gyro_data()
    
    # Subtract the bias to get the corrected gyroscope reading
    gz -= gyro_bias_z

    # Integrate acceleration to get velocity (basic integration)
    vx += ax * dt
    vy += ay * dt
    vz += az * dt

    # Print the results
    with open('data.txt', 'a') as f:
        f.write(f"{ax},{ay},{gz},{vx},{vy}\n")

    # Short delay before next reading
    time.sleep(0.005)
send('m1-000m2-000m3-000m4-000')
time.sleep(1)
send('m1+100m2-100m3-100m4+100')
for i in range(500):
    # Calculate time difference
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Read accelerometer data
    ax, ay, az = get_accel_data()
    
    # Subtract the bias to get the corrected accelerometer readings
    ax -= accel_bias_x
    ay -= accel_bias_y
    az -= accel_bias_z

    # Read gyroscope data (only Z-axis)
    gz = get_gyro_data()
    
    # Subtract the bias to get the corrected gyroscope reading
    gz -= gyro_bias_z

    # Integrate acceleration to get velocity (basic integration)
    vx += ax * dt
    vy += ay * dt
    vz += az * dt

    # Print the results
    with open('data.txt', 'a') as f:
        f.write(f"{ax},{ay},{gz},{vx},{vy}\n")

    # Short delay before next reading
    time.sleep(0.005)
send('m1-000m2-000m3-000m4-000')