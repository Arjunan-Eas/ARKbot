import smbus2
from smbus2 import SMBus, i2c_msg
import time

# MPU6500 Registers
MPU6500_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_ZOUT_H = 0x47

CORRECTION_FACTOR = 2
WINDOW = 50

# Initialize the I2C bus
bus = smbus2.SMBus(1)

def course_correct(direction, amount, motors):
    if direction == "left":
        if((0 + amount < motors[0] < 255 - amount) and (0 + amount < motors[3] < 255 - amount)):
            motors[0] -= amount
            motors[3] -= amount
        if((0 + amount < motors[1] < 255 - amount) and (0 + amount < motors[2] < 255 - amount)):
            motors[1] += amount
            motors[2] += amount
    elif direction == "right":
        if((0 + amount < motors[0] < 255 - amount) and (0 + amount < motors[3] < 255 - amount)):
            motors[0] += amount
            motors[3] += amount
        if((0 + amount < motors[1] < 255 - amount) and (0 + amount < motors[2] < 255 - amount)):
            motors[1] -= amount
            motors[2] -= amount
    print(f'motors: {motors}')
    return motors

def format_motor_values(motors):
    # Create a list of formatted motor values, each with three digits
    return [str(motor).zfill(3) for motor in motors]

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

motors = [100, 100, 100, 100]

send(f'm1-{motors[0]}m2-{motors[1]}m3-{motors[2]}m4-{motors[3]}')
time.sleep(0.5)

rotation_list = []

def main():

    global last_time
    motors = [100, 100, 100, 100]
    for i in range(100):
        # Calculate time difference
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Read gyroscope data (only Z-axis)
        gz = get_gyro_data()
        
        # Subtract the bias to get the corrected gyroscope reading
        gz -= gyro_bias_z

        if(len(rotation_list) > WINDOW):
            rotation_list.pop(0)
        rotation_list.append(gz)

        rolling_avg = (sum(rotation_list) / len(rotation_list))
        
        if( rolling_avg > 0.75):
            print('cor right')
            motors = course_correct("right", CORRECTION_FACTOR, motors)

            l = format_motor_values(motors)

            send(f'm1-{l[0]}m2-{l[1]}m3-{l[2]}m4-{l[3]}')

        elif(rolling_avg < -0.75):
            print('cor left')
            motors = course_correct("left", CORRECTION_FACTOR, motors)

            l = format_motor_values(motors)
            
            print(f'l: {l}')
            send(f'm1-{l[0]}m2-{l[1]}m3-{l[2]}m4-{l[3]}')
        
        # Print the results
        with open('data.txt', 'a') as f:
            f.write(f"{rolling_avg},{motors[0]},{motors[1]},{motors[2]},{motors[3]}\n")

        # Short delay before next reading
        time.sleep(0.005)
    send('m1-000m2-000m3-000m4-000')

main()