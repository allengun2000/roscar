#!/usr/bin/env python
import serial
import time
import numpy as np
import struct
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
# encoder  black =42; white = 43; orange = 44; brown=5V ;blue=0v
# stree_angel  0 12V can1  ;can2 x x x
# stop 4 oil 5v

class getdata:

    ser = None
    port = ''
    baudrate = 57600

    #def __init__(self, port='/dev/cu.SLAB_USBtoUART', baudrate=115200):
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600,timeout=0.5):
   
        print("Initializing usb wheel")
        print("Port:",port)
        self.port = port
        self.baudrate = baudrate
        self.ser = self._connect()

    def _connect(self):
        '''Connects to the serial port with the name `self.port`.'''
        ser = serial.Serial()
        ser.baudrate = 57600
        ser.port = self.port
        try:
            ser.open()
            if ser.is_open:
                print("usb Connected")
        except serial.serialutil.SerialException as err:
            print(err)
            exit(0)
        return ser
    def _close(self):
        self.ser.close()

    def push_cmd(self,stop,oil):
        # print(stop)
        stt=b's'+str(stop)+b'o'+str(oil)+b'\n'
        self.ser.write(stt)
        # self.ser.write(b's')
        # self.ser.write(str(stop))
        # self.ser.write(b'o')
        # self.ser.write(str(oil))
        # self.ser.write(b'\n')

    def run(self):
        self.ser.flushInput()
        while 1:
            c = self.ser.read()
            # print("  ",c)
            if c == b'\n':
                angle=0
                encoder=0
                data_length_byte = self.ser.read(3)
                if data_length_byte == b'48\r':
                    temp = []
                    c=self.ser.read(1)
                    while c!=b'\r':
                        temp.append(c)
                        c = self.ser.read(1)
                    if len(temp)==0:
                        continue
                    if ord(temp[0])-48==-3:
                        for i in range(len(temp)-1):
                            angle=angle-(ord(temp[i+1])-48)*(10**(len(temp)-i-2))
                            
                    else:
                        for i in range(len(temp)):
                            angle=angle+(ord(temp[i])-48)*(10**(len(temp)-i-1))
                    temp = []
                    c=self.ser.read(1)
                    while c!=b'\r':
                        temp.append(c)
                        c = self.ser.read(1)
                    if len(temp)==0:
                        continue
                    for i in range(len(temp)):
                        encoder=encoder+(ord(temp[i])-48)*(10**(len(temp)-i-1))
                        # print(encoder)
                        

                return angle,encoder
            self.ser.flushInput()

def callback(data):
     global oil_msg
     global stop
     if data.x<0:
         oil_msg=0
         stop=1
     else:
         oil_msg=int(data.x*25) #0-255
         stop=0
     


oil_msg=0
stop=0
b_encoder=0
arduino_ser=getdata()
pub = rospy.Publisher('/wheelangle', Float32, queue_size=10) #7800 -7800
pub1 = rospy.Publisher('/speed_encoder', Float32, queue_size=10) #
rospy.Subscriber("/cmd", Pose2D, callback)
rospy.init_node('feedback', anonymous=True)
rate = rospy.Rate(100)
steer=Float32()
encoder=Float32()
while not rospy.is_shutdown():
    b_encoder=encoder.data
    steer.data ,encoder.data=arduino_ser.run()
    steer.data=steer.data
    pub.publish(steer)
    if encoder.data <= 2147483648:
      pass
    else:
      encoder.data = (encoder.data - 4294967296)
    pub1.publish(encoder.data-b_encoder) 
    rate.sleep()
    # print(stop)
    arduino_ser.push_cmd(stop,oil_msg)
arduino_ser._close()
arduino_ser._close()
arduino_ser._close()