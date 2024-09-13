import smbus2
from smbus2 import SMBus, i2c_msg
import pygame
from pygame._sdl2 import controller
import time



# MPU6500 Registers
MPU6500_ADDR = 0x68
GYRO_YOUT_H = 0x45

# PD control parameters
cor_lims = [2]
wins = [10]
motor_speeds = [[100,-100,-100,100]]
angular_speeds = [-150]

# I2C stuff
gyro_bus = smbus2.SMBus(1)
MOTOR_CONTROLLER = 0x08
UPPER_CONTROLLER = 0x09

# Setup controller
game_controller = None
if controller.get_count() > 0:
    game_controller = controller.Controller(0)
    print(f"Controller Name: {game_controller.name}")
else:
    print("No controller connected")
    exit(1)

pygame.init()
controller.init()

# Threshold for detecting significant movement on an axis
MOVEMENT_THRESHOLD = 0.1
JOYSTICK_MAX = 32767
JOYSTICK_DEADZONE = 7500


# ccw [neg, pos, pos, neg]
# cw [pos, neg, neg, pos]
def rotation_cc(rotation_direction, amount, motors):
    if rotation_direction == "ccw" and amount > 0:
        # Motors in + direction
        motors[1] = min(motors[1] + amount, 255)
        motors[2] = min(motors[2] + amount, 255)
        # Motors in - direction
        motors[0] = max(motors[0] - amount, -255)
        motors[3] = max(motors[3] - amount, -255)

    elif rotation_direction == "ccw" and amount < 0:
        motors[1] = max(motors[1] + amount, 0)
        motors[2] = max(motors[2] + amount, 0)

        motors[0] = min(motors[0] - amount, 0)
        motors[3] = min(motors[3] - amount, 0)

    elif rotation_direction == "cw" and amount < 0:
        motors[0] = max(motors[0] + amount, 0)
        motors[3] = max(motors[3] + amount, 0)

        motors[1] = min(motors[1] - amount, 0)
        motors[2] = min(motors[2] - amount, 0)

    elif rotation_direction == "cw" and amount > 0:
        motors[0] = min(motors[0] + amount, 255)
        motors[3] = min(motors[3] + amount, 255)

        motors[1] = max(motors[1] - amount, -255)
        motors[2] = max(motors[2] - amount, -255)
    return motors, amount

def gains(deltas):
    derivative = abs((abs(deltas[-1][0]) - abs(deltas[-2][0])) / (deltas[-1][1] - deltas[-2][1]))
    if(derivative <= 50 and deltas[-1][0] > 1):
        return 0.5, 0.5
    elif((deltas[-1][0]) < 1 and derivative < 50):
        return 0.15, 0.15
    else:
        return 0.75, 0.25

def PD_CF(deltas):
    MAX_CORRECTION = 25


    # Proportional
    prop = 0
    if(deltas[-1][0] > 300):
        prop = MAX_CORRECTION
    elif(deltas[-1][0] < -300):
        prop -MAX_CORRECTION
    else:
        prop = (MAX_CORRECTION * (deltas[-1][0] / 300))
    
    # Derivative
    deriv = 0
    if(len(deltas) > 1):
        Kp, Kd = gains(deltas)
        print(f"Kp: {Kp}  Kd: {Kd}")

        derivative = (abs(deltas[-1][0]) - abs(deltas[-2][0])) / (deltas[-1][1] - deltas[-2][1])
        if(derivative >= 300):
            deriv = MAX_CORRECTION
        elif(derivative <= -300):
            deriv = -MAX_CORRECTION
        else:
            deriv = (MAX_CORRECTION * (derivative / 300))

        with open("data.txt",'a') as f:
            f.write(f'{int(round(Kp * prop + Kd * deriv))},{derivative},{prop},{deriv},')
    else:
        Kp, Kd = 0.75, 0.25
        print(f"Kp: {Kp}  Kd: {Kd}")
        with open("data.txt",'a') as f:
            f.write(f'{0},{0},{0},{0},')
    return int(round(Kp * prop + Kd * deriv))

def format_motor_values(motors):
    # Create a list of formatted motor values, each with three digits
    return [str(motor).zfill(3) for motor in motors]

def send_to_motors(speeds=[0,0,0,0]):
    
    signs = ["","","",""]

    for m in range(4):
        if speeds[m] >= 0:
            signs[m] = "-" # minus because of established front/back convention
        else:
            signs[m] = "+"
    
    speeds = format_motor_values([abs(speed) for speed in speeds])

    message = f"m1{signs[0]}{speeds[0]}m2{signs[1]}{speeds[1]}m3{signs[2]}{speeds[2]}m4{signs[3]}{speeds[3]}"

    with SMBus(7) as bus:
        # Create an I2C message with the data
        msg = i2c_msg.write(MOTOR_CONTROLLER, message)
        # Write the message to the I2C bus
        bus.i2c_rdwr(msg)

def send_to_upper(command):
    with SMBus(7) as bus:
        # Create an I2C message with the data
        msg = i2c_msg.write(UPPER_CONTROLLER, command)
        # Write the message to the I2C bus
        bus.i2c_rdwr(msg)

def read_word_2c(addr):
    high = gyro_bus.read_byte_data(MPU6500_ADDR, addr)
    low = gyro_bus.read_byte_data(MPU6500_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def get_gyro_data():
    gyro_z = read_word_2c(GYRO_YOUT_H) / 131.0  # Scale factor for gyroscope
    return gyro_z

def calibrate_imu(samples=1000):
    accel_bias_x, accel_bias_y, accel_bias_z = 0, 0, 0
    gyro_bias_z = 0
    
    print("Calibrating IMU...")
    for _ in range(samples):
        gz = get_gyro_data()
        
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
gyro_bus.write_byte_data(MPU6500_ADDR, 0x6B, 0)  # Reset the power management register to wake up the MPU6500

# Calibrate the IMU
accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_z = calibrate_imu()

# Initialize velocity components
vx, vy, vz = 0, 0, 0
last_time = time.time()


rotation_list = []

def main(initial_speeds, target, window, correction_limit):
    motors = [initial_speeds[0], initial_speeds[1], initial_speeds[2], initial_speeds[3]]
    deltas = []
    send_to_motors(initial_speeds)
    time.sleep(0.5)

    correction_factors = []

    with open('data.txt', 'w') as f:
        pass

    for i in range(800):

        # Read gyroscope data (only Z-axis)
        gz = get_gyro_data()
        
        
        # Subtract the bias to get the corrected gyroscope reading
        gz -= gyro_bias_z
    

        if(len(rotation_list) > window):
            rotation_list.pop(0)
        rotation_list.append(gz)

        rolling_avg = (sum(rotation_list) / len(rotation_list))
        
        if(target > 0):
            # if(abs(target - rolling_avg) > correction_limit)
            deltas.append(((target - rolling_avg), time.time()))
            motors, amount = rotation_cc("ccw", PD_CF(deltas), motors)
            
            correction_factors.append(amount)
            # print(motors, amount)
            send_to_motors(motors)

        
        elif(target < 0):
            # if(abs(target - rolling_avg) > correction_limit)
            deltas.append(((rolling_avg - target), time.time()))
            motors, amount = rotation_cc("cw", PD_CF(deltas), motors)

            correction_factors.append(amount)
            # print(motors, amount)
            send_to_motors(motors)

        
        # Print the results
        with open('data.txt', 'a') as f:
            f.write(f"{rolling_avg},{motors[0]},{motors[1]},{time.time()}\n")

        # Short delay before next reading
        time.sleep(0.005)
    send_to_motors()
    
