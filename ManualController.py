from datetime import datetime as dt
import serial
import numpy as np
import time

class manualController:
    # Function to initialize the controller
    def __init__(self, port1, port2=None, port3=None, port4=None, port5=None, port6=None, serialRate=2000000):
        self.port1 = port1
        self.port2 = port2
        self.port3 = port3
        self.port4 = port4
        self.port5 = port5
        self.port6 = port6

        # Set up the serial communication with each of the actuators
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

    # Function to take the actuator height from 0->1 and encode it for serial
    def encodeHeight(self, height):
        binHeight = int(3400 * height)

        position = format(binHeight, "b").zfill(12)

        byte1 = int('1' + position[:6] + '1', 2)
        byte2 = int('0' + position[6:] + '1', 2)

        return bytearray([byte1, byte2])

    # Function to send a height command to each of the actuators
    def setHeight(self, h1, h2=None, h3=None, h4=None, h5=None, h6=None):
        self.ser1.write(self.encodeHeight(h1))

        if self.port2 != None:
            self.ser2.write(self.encodeHeight(h2))
        if self.port3 != None:
            self.ser3.write(self.encodeHeight(h3))
        if self.port4 != None:
            self.ser4.write(self.encodeHeight(h4))
        if self.port5 != None:
            self.ser5.write(self.encodeHeight(h5))
        if self.port6 != None:
            self.ser6.write(self.encodeHeight(h6))

    # Function to home the actuators
    def home(self):
        self.setHeight(0, 0, 0, 0, 0, 0)


def main():
    # The serial communication port for each of the actuators
    s1 = '/dev/cu.usbmodem111103'
    s2 = None
    s3 = None
    s4 = None
    s5 = None
    s6 = None

    # The manual controller objects
    controller = manualController(s1, s2, s3, s4, s5, s6)

    # Home the actuator
    controller.home()

    # Send a height command to the actuator
    controller.setHeight(0.5)

if __name__ == "__main__":
    main()
