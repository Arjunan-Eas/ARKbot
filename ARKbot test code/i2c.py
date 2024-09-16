from smbus2 import SMBus, i2c_msg
from pynput import keyboard

# Define the I2C bus and device address
i2c_bus_number = 7  # Change to your I2C bus number if different
i2c_device_address = 0x08  # Change to your device address
pressed_command = []

key_actions = {
    keyboard.Key.up: "m1+000m2+255m3+255m4+000",
    keyboard.Key.down: "m1-255m2-255m3-255m4-255",
}
def to_bytes(string):
    return [ord(char) for char in string]

def on_press(key):
    if key == keyboard.Key.up:
        ahead(key)
    elif key == keyboard.Key.down:
        back(key)

def on_release(key):
    stop(key)

def ahead(key):
    global pressed_command
    pressed_command = to_bytes(key_actions[key])
    send()

def back(key):
    global pressed_command
    pressed_command = to_bytes(key_actions[key])
    send()

def stop(key):
    global pressed_command
    stop = "m1+000m2+000m3+000m4+000"
    pressed_command = to_bytes(stop)
    send()
# Open I2C bus

def send():
    with SMBus(i2c_bus_number) as bus:
        # Create an I2C message with the data
        msg = i2c_msg.write(i2c_device_address, pressed_command)
        
        # Write the message to the I2C bus
        bus.i2c_rdwr(msg)
        
    print("Data sent successfully")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()


