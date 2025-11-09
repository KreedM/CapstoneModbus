import time
import random
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

PORT = "COM4"
BAUDRATE = 9600
POLL_INTERVAL = 1  # seconds

def setup_client():
    client = ModbusSerialClient(
        PORT,
        framer = FramerType.RTU,
        baudrate = BAUDRATE,
        bytesize = 8,
        parity = "N",
        stopbits = 1,
    )
    if not client.connect():
        raise ConnectionError(f"Could not connect to {PORT}")
    
    return client

def read(client, address):
    rr = client.read_holding_registers(0, count = 123, device_id = address)
    if rr.isError():
        print(f"[{address}] Read error: {rr}")

        return None

    return rr.registers

def write(client, address):
    values = [random.randint(0, 1000) for _ in range(123)]

    print("Writing values: " + str(values))
    
    wr = client.write_registers(0, values, device_id = address)
    if wr.isError():
        print(f"[{address}] Write error: {wr}")

        return None

def main():
    client = setup_client()

    print("Starting logging... (Ctrl+C to stop)")

    try:
        while True:
            write(client, 0x42)

            print("Reading values: " + str(read(client, 0x42)))

            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        client.close()

if __name__ == "__main__":
    main()
