from smbus2 import SMBus, i2c_msg
import time
import struct


# Use I2C bus 7 (or adjust based on your Jetson's configuration)
bus_num = 7
bus = SMBus(bus_num)

# Arduino addresses
address_8 = 0x08  # Arduino Nano at address 8 (write only)
address_9 = 0x09  # Arduino Nano at address 9 (read and write)

def send(device, message):
    with SMBus(bus_num) as bus:
        # Convert the message to bytes
        msg = i2c_msg.write(device, message.encode('utf-8'))
        bus.i2c_rdwr(msg)
        
    print(f"Data sent to device {device} successfully")

def read_float_from_device(device):
    with SMBus(bus_num) as bus:
        read_msg = i2c_msg.read(device, 4)  # Read 4 bytes for a float
        bus.i2c_rdwr(read_msg)
        
        received_bytes = bytes(list(read_msg))
        # Unpack the bytes into a float
        received_float = struct.unpack('f', received_bytes)[0]
        return received_float

try:
    
    # Write to Arduino at address 8
    send(address_8, f'm1-200m2-200m3-200m4-200')
    print("Driving at 200")
    time.sleep(3)
    send(address_8, f'm1-000m2-000m3-000m4-000')
    print("Stopped motors")
    time.sleep(1)

    # Write to Arduino at address 9
    send(address_9, "openClaw,200,100")
    print("Open claw")
    time.sleep(1)
    send(address_9, "closeClaw,200,100")
    print("Close claw")
    time.sleep(1)

    # Write to Arduino at address 9
    send(address_9, "armUp,200,100")
    print("Arm up")
    time.sleep(1)
    send(address_9, "armDown,200,50")
    print("Arm down")
    time.sleep(1)

    # Read from Arduino at address 9
    received_string = read_float_from_device(address_9)
    print(f"Distance: {received_string}")

finally:
    bus.close()
