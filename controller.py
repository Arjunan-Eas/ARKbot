import pygame
from smbus2 import SMBus, i2c_msg
from pygame._sdl2 import controller
import time

# Initialize Pygame and the SDL2 controller module
pygame.init()
controller.init()

# Define the I2C bus and device address
i2c_bus_number = 7  # Change to your I2C bus number if different
i2c_device_address = 0x08  # Change to your device address

# Rate limiting parameters
messages_per_second = 50  # Limit to 10 messages per second
min_time_between_sends = 1.0 / messages_per_second  # Minimum time between sends
last_send_time = 0  # Initialize last send time

def send(command):
    global last_send_time
    current_time = time.time()
    
    # Check if enough time has passed since the last send
    if current_time - last_send_time >= min_time_between_sends:
        with SMBus(i2c_bus_number) as bus:
            # Create an I2C message with the data
            msg = i2c_msg.write(i2c_device_address, command)
            # Write the message to the I2C bus
            bus.i2c_rdwr(msg)
        print("Data sent successfully")
        
        # Update last send time
        last_send_time = current_time
    else:
        print("Rate limit exceeded, skipping send")

# Setup controller
game_controller = None
if controller.get_count() > 0:
    game_controller = controller.Controller(0)
    print(f"Controller Name: {game_controller.name}")
else:
    print("No controller connected")
    exit(1)

# Threshold for detecting significant movement on an axis
MOVEMENT_THRESHOLD = 0.1
JOYSTICK_MAX = 32767

def to_byte_list(string):
    return [ord(char) for char in string]

# Define all the driving modes
def drive_forward(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1-{value}m2-{value}m3-{value}m4-{value}"
    command = to_byte_list(command_str)
    send(command)

def reverse(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1+{value}m2+{value}m3+{value}m4+{value}"
    command = to_byte_list(command_str)
    send(command)

def left_in_place():
    value = '255'
    command_str = f"m1+{value}m2-{value}m3-{value}m4+{value}"
    command = to_byte_list(command_str)
    send(command)

def right_in_place():
    value = '255'
    command_str = f"m1-{value}m2+{value}m3+{value}m4-{value}"
    command = to_byte_list(command_str)
    send(command)

def stop():
    value = '000'
    command_str = f"m1-{value}m2+{value}m3+{value}m4-{value}"
    command = to_byte_list(command_str)
    send(command)

def standard_right(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1-{value}m2+{value}m3+000m4+000"
    command = to_byte_list(command_str)
    send(command)

def standard_left(value):
    value = str(round((-value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1+{value}m2-{value}m3+000m4+000"
    command = to_byte_list(command_str)
    send(command)

def parallel_right():
    value = '255'
    command_str = f"m1-{value}m2+{value}m3-{value}m4+{value}"
    command = to_byte_list(command_str)
    send(command)

def parallel_left():
    value = '255'
    command_str = f"m1+{value}m2-{value}m3+{value}m4-{value}"
    command = to_byte_list(command_str)
    send(command)

def diagonal_right():
    value = '255'
    command_str = f"m1-{value}m2+000m3-{value}m4+000"
    command = to_byte_list(command_str)
    send(command)

def diagonal_left():
    value = '255'
    command_str = f"m1+000m2-{value}m3+000m4-{value}"
    command = to_byte_list(command_str)
    send(command)

# Define callback functions
def on_button_pressed(button):
    if button == 9:
        left_in_place()
    elif button == 10:
        right_in_place()
    elif button == 13:
        parallel_left()
    elif button == 14:
        parallel_right()
    elif button == 11:
        diagonal_right()
    elif button == 12:
        diagonal_left()

    print(f"Button {button} pressed!")

def on_button_released(button):
    stop()
    print(f"Button {button} released!")

def on_axis_motion(axis, value):
    if axis == 5:
        drive_forward(value)
    elif axis == 4:
        reverse(value)
    elif axis == 0:
        if value < 0:
            standard_left(value)
        else:
            standard_right(value)
        print(f"Axis {axis} moved to {value}")

def on_drive_and_turn(value1, value5):
    print(f"Both Axis 1 and Axis 5 are being moved! (Axis 1: {value1}, Axis 5: {value5})")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.CONTROLLERAXISMOTION:
            on_axis_motion(event.axis, event.value)

        elif event.type == pygame.CONTROLLERBUTTONDOWN:
            on_button_pressed(event.button)

        elif event.type == pygame.CONTROLLERBUTTONUP:
            on_button_released(event.button)

if game_controller:
    game_controller.quit()
controller.quit()
pygame.quit()
