import pygame
from bleak import BleakClient
from pygame._sdl2 import controller
import asyncio

# Initialize Pygame and the SDL2 controller module
pygame.init()
controller.init()

# Define the Bluetooth device address and characteristic UUID
DEVICE_ADDRESS = "D4:D4:DA:96:FF:92"  # Replace with your device's address
CHARACTERISTIC_UUID = "0b9f074e-0882-4702-ba43-40d726a0dddc"  # Replace with your characteristic UUID

# Define a global variable for the BleakClient
bleak_client = None

# Connect to the Bluetooth device
async def connect_to_device(address):
    global bleak_client
    try:
        bleak_client = BleakClient(address, timeout=30.0)
        await bleak_client.connect()
        print(f"Connected: {bleak_client.is_connected}")
    except Exception as e:
        print(f"Failed to connect: {e}")

# Send data over Bluetooth
async def send(command):
    if bleak_client and bleak_client.is_connected:
        await bleak_client.write_gatt_char(CHARACTERISTIC_UUID, command)
        print("Data sent successfully")
    else:
        print("Bluetooth device not connected")

# Convert a string to bytes
def to_bytes(string):
    return string.encode('utf-8')

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

# Define all the driving modes
async def drive_forward(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1-{value}m2-{value}m3-{value}m4-{value}"
    command = to_bytes(command_str)
    await send(command)

async def reverse(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1+{value}m2+{value}m3+{value}m4+{value}"
    command = to_bytes(command_str)
    await send(command)

async def left_in_place():
    value = '255'
    command_str = f"m1+{value}m2-{value}m3-{value}m4+{value}"
    command = to_bytes(command_str)
    await send(command)

async def right_in_place():
    value = '255'
    command_str = f"m1-{value}m2+{value}m3+{value}m4-{value}"
    command = to_bytes(command_str)
    await send(command)

async def stop():
    value = '000'
    command_str = f"m1-{value}m2+{value}m3+{value}m4-{value}"
    command = to_bytes(command_str)
    await send(command)

async def standard_right(value):
    value = str(round((value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1-{value}m2+{value}m3+000m4+000"
    command = to_bytes(command_str)
    await send(command)

async def standard_left(value):
    value = str(round((-value / JOYSTICK_MAX) * 255))
    while len(value) < 3:
        value = '0' + value
    command_str = f"m1+{value}m2-{value}m3+000m4+000"
    command = to_bytes(command_str)
    await send(command)

async def parallel_right():
    value = '255'
    command_str = f"m1-{value}m2+{value}m3-{value}m4+{value}"
    command = to_bytes(command_str)
    await send(command)

async def parallel_left():
    value = '255'
    command_str = f"m1+{value}m2-{value}m3+{value}m4-{value}"
    command = to_bytes(command_str)
    await send(command)

async def diagonal_right():
    value = '255'
    command_str = f"m1-{value}m2+000m3-{value}m4+000"
    command = to_bytes(command_str)
    await send(command)

async def diagonal_left():
    value = '255'
    command_str = f"m1+000m2-{value}m3+000m4-{value}"
    command = to_bytes(command_str)
    await send(command)

# Define callback functions
async def on_button_pressed(button):
    if button == 9:
        await left_in_place()
    elif button == 10:
        await right_in_place()
    elif button == 13:
        await parallel_left()
    elif button == 14:
        await parallel_right()
    elif button == 11:
        await diagonal_right()
    elif button == 12:
        await diagonal_left()

    print(f"Button {button} pressed!")

async def on_button_released(button):
    await stop()
    print(f"Button {button} released!")

async def on_axis_motion(axis, value):
    if axis == 5:
        await drive_forward(value)
    elif axis == 4:
        await reverse(value)
    elif axis == 0:
        if value < 0:
            await standard_left(value)
        else:
            await standard_right(value)
        print(f"Axis {axis} moved to {value}")

async def on_drive_and_turn(value1, value5):
    print(f"Both Axis 1 and Axis 5 are being moved! (Axis 1: {value1}, Axis 5: {value5})")

# Main event loop
async def main():
    await connect_to_device(DEVICE_ADDRESS)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.CONTROLLERAXISMOTION:
                await on_axis_motion(event.axis, event.value)

            elif event.type == pygame.CONTROLLERBUTTONDOWN:
                await on_button_pressed(event.button)

            elif event.type == pygame.CONTROLLERBUTTONUP:
                await on_button_released(event.button)

    if bleak_client:
        await bleak_client.disconnect()
        print("Disconnected")

if __name__ == "__main__":
    asyncio.run(main())
    controller.quit()
    pygame.quit()
