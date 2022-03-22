


class Table:
    def __init__(self):
        self.BaseFrame = np.array([0, 0, 0])        # The Coordinate for the base reference frame



from datetime import datetime as dt
import serial
import numpy as np
import time

def encodeHeight(height):
    binHeight = int(3400 * height)

    position = format(binHeight, "b").zfill(12)

    byte1 = int('1' + position[:6] + '1', 2)
    byte2 = int('0' + position[6:] + '1', 2)

    return bytearray([byte1, byte2])

def main(ser1, ser2, ser3, ser4, ser5, ser6):
    startTime = dt.now()
    elapsedTime = 0


    while elapsedTime <= 60:
        elapsedTime = float((dt.now() - startTime).total_seconds())

        s1 = np.sin(4*elapsedTime) * 0.03 + 0.03;
        s2 = np.sin(4*elapsedTime + np.pi/3) * 0.03 + 0.03;
        s3 = np.sin(4*elapsedTime + np.pi * 2/3) * 0.03 + 0.03;

        ser1.write(encodeHeight(s1))
        ser2.write(encodeHeight(s1))
        ser3.write(encodeHeight(s2))
        ser4.write(encodeHeight(s2))
        ser5.write(encodeHeight(s3))
        ser6.write(encodeHeight(s3))

        time.sleep(0.2)

if __name__ == "__main__":
    ser1 = serial.Serial('/dev/cu.usbmodem111103', 2000000, timeout=1)
    ser2 = serial.Serial('/dev/cu.usbmodem1114403', 2000000, timeout=1)
    ser3 = serial.Serial('/dev/cu.usbmodem111203', 2000000, timeout=1)
    ser4 = serial.Serial('/dev/cu.usbmodem1114203', 2000000, timeout=1)
    ser5 = serial.Serial('/dev/cu.usbmodem111303', 2000000, timeout=1)
    ser6 = serial.Serial('/dev/cu.usbmodem1114303', 2000000, timeout=1)

    ser1.write(encodeHeight(0))
    ser2.write(encodeHeight(0))
    ser3.write(encodeHeight(0))
    ser4.write(encodeHeight(0))
    ser5.write(encodeHeight(0))
    ser6.write(encodeHeight(0))

    time.sleep(3)

    main(ser1, ser2, ser3, ser4, ser5, ser6)

    ser1.write(encodeHeight(0))
    ser2.write(encodeHeight(0))
    ser3.write(encodeHeight(0))
    ser4.write(encodeHeight(0))
    ser5.write(encodeHeight(0))
    ser6.write(encodeHeight(0))
