import cv2
from ultralytics import YOLO
import smbus2
import time

# MPU6500 Registers
MPU6500_ADDR = 0x68
GYRO_YOUT_H = 0x45

FRAME_INTERVAL = 30
frame_count = 0

grab_position = [(0, 135, 60, 270)]

cor_lims = [2]
wins = [10]
motor_speeds = [[100,-100,-100,100]]
angular_speeds = [-150]

# I2C Constants
arduino_bus_number = 7
UPPER_ARDUINO = 9
LOWER_ARDUINO = 8
gyro_bus = smbus2.SMBus(1)

def rotation(rotation_direction, amount, motors):
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

        with open("data.txt",'a') as f:
            f.write(f'{0},{0},{0},{0},')
    return int(round(Kp * prop + Kd * deriv))

def format_motor_values(motors):
    # Create a list of formatted motor values, each with three digits
    return [str(motor).zfill(3) for motor in motors]

def send(device=8, message="", speeds=[0,0,0,0]):
    if device == 8:
        signs = ["","","",""]

        for m in range(4):
            if speeds[m] >= 0:
                speeds[m] = min(255, speeds[m])
                signs[m] = "-" # minus because of established front/back convention
            else:
                speeds[m] = max(-255, speeds[m])
                signs[m] = "+"

        speeds = format_motor_values([abs(speed) for speed in speeds])
        message = f"m1{signs[0]}{speeds[0]}m2{signs[1]}{speeds[1]}m3{signs[2]}{speeds[2]}m4{signs[3]}{speeds[3]}"

    with smbus2.SMBus(7) as bus:
        # Create an I2C message with the data
        msg = smbus2.i2c_msg.write(device, message)
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
    gyro_bias_z = 0
    print("Calibrating IMU...")
    for _ in range(samples):
        gz = get_gyro_data()
        
        gyro_bias_z += gz
        
        time.sleep(0.002)  # Small delay to avoid overwhelming the I2C bus

    # Calculate average bias
    gyro_bias_z /= samples
    
    print(f"Gyro Bias Z: {gyro_bias_z:.3f}")
    
    return gyro_bias_z


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=480,
    capture_height=270,
    display_width=480,
    display_height=270,
    framerate=15,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def get_object_coordinates(results):
    """
    Returns a list of coordinates for detected objects in the form of (x1, y1, x2, y2).
    """
    coordinates = []
    for box in results[0].boxes:
        # Extract the bounding box coordinates
        x1, y1, x2, y2 = box.xyxy[0]
        coordinates.append((int(x1), int(y1), int(x2), int(y2)))
    return coordinates


def turn(target, duration=0):
    WINDOW = 10
    rotation_list = []
    deltas = []
    motors = [0,0,0,0]
    send()
    rolling_avg = 0

    while abs(rolling_avg) < abs(target):
        gz = get_gyro_data()
        gz -= gyro_bias_z
        
        if(len(rotation_list) > WINDOW):
            rotation_list.pop(0)
        rotation_list.append(gz)

        rolling_avg = (sum(rotation_list) / len(rotation_list))
            
        if(target > 0):
            deltas.append(((target - rolling_avg), time.time()))
            motors, amount = rotation("ccw", PD_CF(deltas), motors)
            send(speeds=motors)
        
        elif(target < 0):
            deltas.append(((rolling_avg - target), time.time()))
            motors, amount = rotation("cw", PD_CF(deltas), motors)
            send(speeds=motors)
        
        time.sleep(0.005)
    
    time.sleep(duration)

    send()


def seek():
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to read the frame.")
        return True, []
    
    # Run YOLOv8 detection on the current frame
    results = model(frame)
    
    # Get the coordinates of detected objects
    object_coordinates = get_object_coordinates(results)
    
    if object_coordinates:
        return False, object_coordinates, results
    else:
        turn(60)
        return True, object_coordinates, results


def approach(coord_delta):

    # Try changing time moving forward, tweaking directions, using coord2 vs coord4
    motors = [100, 100, 100, 100]

    if coord_delta[4] > 1000:

        if coord_delta[2] > 200:
            print("Adjusting left")
            motors[1] += round(50 * coord_delta[2] / 200)
            motors[2] += round(50 * coord_delta[2] / 200)
            motors[0] -= round(coord_delta[2] / 200)
            motors[3] -= round(coord_delta[2] / 200)

        elif coord_delta[2] < 200:
            print("Adjusting right")
            motors[0] -= round(50 * coord_delta[2] / 200)
            motors[3] -= round(50 * coord_delta[2] / 200)
            motors[1] += round(coord_delta[2] / 200)
            motors[2] += round(coord_delta[2] / 200)
    
        else:
            print("Continuing straight")
            motors[1] += round(30 * coord_delta[4] / 4500)
            motors[2] += round(30 * coord_delta[4] / 4500)
            motors[0] += round(20 * coord_delta[4] / 4500)
            motors[3] += round(20 * coord_delta[4] / 4500)
        send(speeds=motors)
        time.sleep(0.25)
        send()
    else:
        if coord_delta[2] > 75 and coord_delta[4] < 250:
            print("Adjusting left in place")
            turn(15)

        elif coord_delta[2] < -75 and coord_delta[4] < 250:
            print("Adjusting right in place")
            turn(-15)
    
        else:
            print("Continuing straight slowly")
            motors[1] = 80
            motors[2] = 80
            motors[0] = 90
            motors[3] = 90
            
            send(speeds=motors)
            time.sleep(0.25)
            send()
    
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to read the frame.")
        return True, []
    
    # Run YOLOv8 detection on the current frame
    results = model(frame)
    
    # Get the coordinates of detected objects
    object_coordinates = get_object_coordinates(results)

    if object_coordinates:
        coordinate_delta = [grab_position[0][i] - object_coordinates[0][i] for i in range(4)]
        grab_box_area = ((grab_position[0][2] - grab_position[0][0]) * (grab_position[0][3] - grab_position[0][1]))
        object_box_area = ((object_coordinates[0][2] - object_coordinates[0][0]) * (object_coordinates[0][3] - object_coordinates[0][1]))
        coordinate_delta.append(grab_box_area - object_box_area)
        print(coordinate_delta)
    else:
        print("Lost object!!!")
        return True, coord_delta, results, True

    if coord_delta[4] <= 0 and coordinate_delta[4] >= 0:

        if (False not in [True if abs(coordinate_delta[i]) <= 30 else False for i in range(4)]):
            if coordinate_delta[0] == 0 and coordinate_delta[3] == 270:
                print("Reached ball!")
                not_at_ball = False
            else:
                print("Not at ball")
                not_at_ball = True
        else:
            print("Not at ball")
            not_at_ball = True

    if (False not in [True if abs(coordinate_delta[i]) <= 30 else False for i in range(4)]) and (-3500 < coordinate_delta[4] < 3500):
        if coordinate_delta[0] <= 2 and coordinate_delta[3] <= 2:
            print("Reached ball!")
            not_at_ball = False
        else:
            print("Not at ball")
            not_at_ball = True

    else:
        print("Not at ball")
        not_at_ball = True
    
    return not_at_ball, coordinate_delta, results, False

def seeking_process():
    ball_not_detected = True
    global frame_count

    while ball_not_detected:
        if frame_count == FRAME_INTERVAL: 
            ball_not_detected, coords, results = seek()

            annotated_frame = results[0].plot()
            cv2.imwrite("last_ball_frame.jpg", annotated_frame)

            frame_count = 0
        else:
            frame_count += 1
            ret, frame = cap.read()

    coordinate_delta = [grab_position[0][i] - coords[0][i] for i in range(4)]

    grab_box_area = ((grab_position[0][2] - grab_position[0][0]) * (grab_position[0][3] - grab_position[0][1]))
    object_box_area = ((coords[0][2] - coords[0][0]) * (coords[0][3] - coords[0][1]))
    coordinate_delta.append(grab_box_area - object_box_area)
    print(f'Coordinate delta: {coordinate_delta}')

    if (False not in [True if abs(coordinate_delta[i]) <= 30 else False for i in range(4)]) and (-3500 < coordinate_delta[4] < 3500):
        if coordinate_delta[0] <= 2 and coordinate_delta[3] <= 2:
            print("Reached ball!")
            not_at_ball = False
        else:
            print("Not at ball")
            not_at_ball = True
    else:
        print("Not at ball")
        not_at_ball = True
    
    return coordinate_delta, not_at_ball, results

def approaching_process(coordinate_delta, not_at_ball, lost):
    global frame_count

    while not_at_ball:
        if lost:
            coordinate_delta, not_at_ball, results = seeking_process()
            lost = False
        elif frame_count == FRAME_INTERVAL: 
            not_at_ball, coordinate_delta, results, lost = approach(coordinate_delta)
            frame_count = 0

            annotated_frame = results[0].plot()
            cv2.imwrite("last_ball_frame.jpg", annotated_frame)
        else:
            frame_count += 1
            ret, frame = cap.read()

def capture_ball():
    send(UPPER_ARDUINO, "closeClaw,200")
    turn(60, 0.8)

    send(LOWER_ARDUINO, speeds=[100,90,90,100])
    time.sleep(3)
    send()
    send(UPPER_ARDUINO, "openClaw,200,150")

if __name__ == "__main__":

    try:
        
        # Wake up the MPU6500
        gyro_bus.write_byte_data(MPU6500_ADDR, 0x6B, 0)  # Reset the power management register to wake up the MPU6500

        # Calibrate the IMU
        gyro_bias_z = calibrate_imu()

        # Start video stream
        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        # cap = cv2.VideoCapture(0)

        # Load the pre-trained YOLOv8 model
        model = YOLO('arkmodel3.pt')

        # Check if the video capture feed is open
        if not cap.isOpened():
            print("Error: Could not open video stream.")
            exit()

        coordinate_delta, not_at_ball, results = seeking_process()

        approaching_process(coordinate_delta, not_at_ball, False)

        capture_ball()

        ret, frame = cap.read()
        results = model(frame)
        annotated_frame = results[0].plot()
        cv2.imwrite("final_frame.jpg", annotated_frame)

        cap.release()
        cv2.destroyAllWindows()

    except OSError or KeyboardInterrupt or TypeError as e:
        ret, frame = cap.read()
        results = model(frame)
        annotated_frame = results[0].plot()
        cv2.imwrite("final_frame.jpg", annotated_frame)
        print(e)
        cap.release()
        cv2.destroyAllWindows()
        send()