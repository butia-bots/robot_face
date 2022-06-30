#!/usr/bin/env python3

from __future__ import division
import rospy
import time
import threading
from std_msgs.msg import (Int16MultiArray, Int16)
import map as mp

# Define normal statics parameters
h = 50
frequency = 3.5

#animationUpper = float(3)
animationUpper = 0
#animationDown = float (3)
animationDown = 0

class eyelidEnable():
    def __init__(self):
        pub = rospy.Publisher('eyelid', Int16MultiArray, queue_size=10)
        rospy.init_node('eyelidEnable', anonymous=False)
        self.sub_eyelid_st = rospy.Subscriber('emotion', Int16, self.getEyelid_st)
        self.sub_eyelid_dn = Int16MultiArray()
        self.sub_eyelid_dn.data = []   
        self.sub_eyelid_dn = rospy.Subscriber('eye', Int16MultiArray, self.getEyelid_dn)
        rate = rospy.Rate(30)

        # Define the output vector
        self.output = Int16MultiArray()
        self.output.data = []

        self.y = 50
        self.animation = 0
        self.upper = 0
        self.down = 0

        # Start blink thread
        blinkLoop = threading.Thread(name = 'blink', target = eyelidEnable.blink, args = (self,))
        blinkLoop.setDaemon(True)
        blinkLoop.start()

        while not rospy.is_shutdown():
            eyelidEnable.getOutput(self)
            pub.publish(self.output)
            rate.sleep()
        
    def getOutput(self):
        if (self.animation == 1):
            pass
        elif(self.animation == 0): 
            eyelidEnable.setValues(self)
    
    def setValues(self):
        if(self.y > 50):
            self.upper = (h + self.y*2 - 140)
            self.down = 140
            self.output.data = [self.upper, self.upper, self.down, self.down]
        elif(self.y < 50):
            self.upper = h
            self.down = 140 - (h + self.y)
            self.output.data = [self.upper, self.upper, self.down, self.down]
        elif(self.y == 50):
            self.upper = h
            self.down = h
            self.output.data = [self.upper, self.upper, self.down, self.down]

    def getEyelid_dn(self, msg):
        self.data = msg.data
        self.y = 50

    def getEyelid_st(self, msg):
        global h
        global frequency
        self.data = msg.data
        if(self.data == 0):     #Standard
            h = 60 # 60
            frequency = 2
        elif(self.data == 1):   #Happy
            h = 80 # 70
            frequency = 3
        elif(self.data == 2):   #Sad
            h = 20 # 50
            frequency = 4
        elif(self.data == 3):   #Rage
            h = 10 # 45
            frequency = 3
        elif(self.data == 4):   #Scared
            h = 90 # 80
            frequency = 1
    
    def blink(self):
        while(True):
            self.animation = 1
            eyelidEnable.setValues(self)

            time.sleep(0.3)
            saveup = self.upper
            savedown = self.down
            self.output.data = [0, 0, 0, 0]

            time.sleep(0.3)

            self.output.data = [saveup, saveup, savedown, savedown]

            self.animation = 0
            time.sleep(frequency)  
            

if __name__ == '__main__':
    try:
        eyelidEnable()
    except rospy.ROSInterruptException:
        pass