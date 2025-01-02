'''
based on Adafruit Demo: 
https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/examples/bno055_simpletest.py

https://github.com/adafruit/Adafruit_BNO055
https://github.com/adafruit/Adafruit_CircuitPython_BNO055

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython
'''

import struct
import asyncio
import busio                        # type: ignore
import board                        # type: ignore
import adafruit_bno055              # type: ignore
from usb_cdc import data as ser     # type: ignore


SCL_pin = board.GP19
SDA_pin = board.GP18

refresh_rate = 100  # Hz
FRAME_MARKER = b'\xAA\xAA'

i2c = busio.I2C(SCL_pin, SDA_pin, frequency=400000)
bno055 = adafruit_bno055.BNO055_I2C(i2c)

async def read_sensor_data():
    # Pack sensor data into a bytearray
    data = struct.pack('f'*23,  # Format string for struct.pack (23 floating point numbers)
        float(bno055.temperature),
        *bno055.acceleration,
        *bno055.magnetic,
        *bno055.gyro,
        *bno055.euler,
        *bno055.quaternion,
        *bno055.linear_acceleration,
        *bno055.gravity)
    return data


async def send_data(data):
    """Send data and return the number of bytes sent."""
    data = FRAME_MARKER + data
    ser.write(data)
    return len(data)


async def main():
    while True:
        sensor_data = await read_sensor_data()
        await send_data(sensor_data)
        await asyncio.sleep(1/refresh_rate)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"An error occurred: {e}")
