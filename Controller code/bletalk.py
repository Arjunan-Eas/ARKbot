import asyncio
from bleak import BleakClient, BleakScanner

# Replace with your device's address and UUIDs
DEVICE_ADDRESS = "D4:D4:DA:96:FF:92"  # Replace with your device's address
SERVICE_UUID = "f72da735-95fa-4ab5-9d10-ed05901d40c4"  # Replace with the service UUID
CHARACTERISTIC_UUID = "0b9f074e-0882-4702-ba43-40d726a0dddc"  # Replace with the characteristic UUID

async def scan_for_device():
    print("Scanning for devices...")
    devices = await BleakScanner.discover(timeout=20.0)  # Increase scan time
    for device in devices:
        print(f"Found device: {device.address} ({device.name})")
        if device.address == DEVICE_ADDRESS:
            return device.address
    return None

async def connect_to_device(address):
    try:
        async with BleakClient(address, timeout=30.0) as client:  # Increased timeout
            print(f"Connected: {client.is_connected}")

            # Example: Read from a characteristic
            # data = await client.read_gatt_char(CHARACTERISTIC_UUID)
            # print(f"Received data: {data}")

            # Example: Write to a characteristic (uncomment if needed)
            message = "Hello, jelker!"
            byte_data = message.encode('utf-8')  # Encode the string to bytes
            await client.write_gatt_char(CHARACTERISTIC_UUID, byte_data)

            # Keep the connection alive for demonstration
            print("Press Ctrl+C to exit")
            while True:
                await asyncio.sleep(1)
    except asyncio.TimeoutError:
        print("Connection timed out")
    except Exception as e:
        print(f"Failed to connect: {e}")

async def main():
    address = await scan_for_device()
    if address:
        await connect_to_device(address)
    else:
        print("Device not found")

if __name__ == "__main__":
    asyncio.run(main())