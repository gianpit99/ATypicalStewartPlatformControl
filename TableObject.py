from datetime import datetime as dt
import serial
import numpy as np
import time

# Encode the height for serial to the actuator
def encodeHeight(height):
    binHeight = int(3400 * height)

    position = format(binHeight, "b").zfill(12)

    byte1 = int('1' + position[:6] + '1', 2)
    byte2 = int('0' + position[6:] + '1', 2)

    return bytearray([byte1, byte2])


class manualController:
    def __init__(self, port1, port2=None, port3=None, port4=None, port5=None, port6=None, serialRate=2000000):
        self.ser1 = serial.Serial(port1, serialRate, timeout=1)

        if port2 != None:
            self.ser2 = serial.Serial(port2, serialRate, timeout=1)
        if port3 != None:
            self.ser3 = serial.Serial(port3, serialRate, timeout=1)
        if port4 != None:
            self.ser4 = serial.Serial(port4, serialRate, timeout=1)
        if port5 != None:
            self.ser5 = serial.Serial(port5, serialRate, timeout=1)
        if port6 != None:
            self.ser6 = serial.Serial(port6, serialRate, timeout=1)

    def encodeHeight(self, height):
        binHeight = int(3400 * height)

        position = format(binHeight, "b").zfill(12)

        byte1 = int('1' + position[:6] + '1', 2)
        byte2 = int('0' + position[6:] + '1', 2)

        return bytearray([byte1, byte2])

    def setHeight(h1, h2=None, h3=None, h4=None, h5=None, h6=None):
        self.ser1.write(encodeHeight(h1))

        if port2 != None:
            self.ser2.write(encodeHeight(h2))
        if port3 != None:
            self.ser3.write(encodeHeight(h3))
        if port4 != None:
            self.ser4.write(encodeHeight(h4))
        if port5 != None:
            self.ser5.write(encodeHeight(h5))
        if port6 != None:
            self.ser6.write(encodeHeight(h6))


    def encode()

def main(ser1, ser2, ser3, ser4, ser5, ser6):
    startTime = dt.now()
    elapsedTime = 0


    while elapsedTime <= 60:
        elapsedTime = float((dt.now() - startTime).total_seconds())

        s1 = np.sin(4*elapsedTime) * 0.03 + 0.03;
        s2 = np.sin(2*elapsedTime + np.pi/3) * 0.03 + 0.03;
        s3 = np.sin(2*elapsedTime + np.pi * 2/3) * 0.03 + 0.03;

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
