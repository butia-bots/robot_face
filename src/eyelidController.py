#!/usr/bin/env python3

from __future__ import division
import rospy
import time
import threading
from std_msgs.msg import (Int16MultiArray, Int16)
import map as mp

# Define normal statics parameters
h = 50
rightU_h = 50
leftU_h = 50
rightD_h = 50
leftD_h = 50
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
        rate = rospy.Rate(30) # 80hz

        # Define the output vector
        self.output = Int16MultiArray()
        self.output.data = []

        self.y = 50
        self.animation = 0
        self.upper = 0
        self.down = 0
        self.upper_right = 0
        self.upper_left = 0
        self.down_right = 0
        self.down_left = 0

        # Start blink thread
        blinkLoop = threading.Thread(name = 'blink', target = eyelidEnable.blink, args = (self,))
        blinkLoop.setDaemon(True)
        blinkLoop.start()

        while not rospy.is_shutdown():
            eyelidEnable.getOutput(self)
            #output.data = []
            #output.data = [self.upper, self.upper, self.down, self.down]
            rospy.loginfo(self.output)
            pub.publish(self.output)
            rate.sleep()
        
    def getOutput(self):
        if (self.animation == 1):
            print("Piscando...")
        elif(self.animation == 0): 
            eyelidEnable.setValues(self)

    
    def setValues(self):
        # 4 - EyelidRightUp [0]
        # 5 - EyelidLeftUp [1]
        # 6 - EyelidRightDown [2] 
        # 7 - EyelidLeftDown [3]
        if(self.y > 50):
            self.upper_right = (rightU_h + self.y*2 - 140)
            self.upper_left = (leftU_h + self.y*2 - 140)
            self.down_right = 140
            self.down_left = 140
            self.output.data = [self.upper_right, self.upper_left, self.down_right, self.down_left]
        elif(self.y < 50):
            self.upper_right = rightU_h
            self.upper_left = leftU_h
            self.down_right = (140-(rightD_h + self.y))
            self.down_left = (140-(leftD_h + self.y))
            #self.down = 100 - (h + self.y)
            self.output.data = [self.upper_right, self.upper_left, self.down_right, self.down_left]
        elif(self.y == 50):
            # self.upper = h
            self.upper_right = rightU_h
            self.upper_left = leftU_h
            # self.down = h
            self.down_right = rightD_h
            self.down_left = leftD_h
            self.output.data = [self.upper_right, self.upper_left, self.down_right, self.down_left]

    def getEyelid_dn(self, msg):
        self.data = msg.data
        self.y = 50 #- 50
        print(self.data)

    def getEyelid_st(self, msg):
        global h

        global rightU_h
        global leftU_h
        global rightD_h
        global leftD_h

        global frequency
        self.data = msg.data
        if(self.data == 0):     #Standard
            # h = 60
            rightU_h = 60
            leftU_h = 100
            rightD_h = 60
            leftD_h = 70
            frequency = 2
        elif(self.data == 1):   #Happy
            # h = 70
            rightU_h = 60
            leftU_h = 60
            rightD_h = 60
            leftD_h = 60
            frequency = 3
        elif(self.data == 2):   #Sad
            # h = 50
            rightU_h = 45
            leftU_h = 60
            rightD_h = 60
            leftD_h = 60
            frequency = 4
        elif(self.data == 3):   #Rage
            # h = 45
            rrightU_h = 45
            leftU_h = 60
            rightD_h = 60
            leftD_h = 60
            frequency = 3
        elif(self.data == 4):   #Scared
            # h = 80
            rightU_h = 130
            leftU_h = 130
            rightD_h = 130
            leftD_h = 130
            frequency = 1
        #print(self.data)
    
    def blink(self):
        while(True):
            self.animation = 1
            eyelidEnable.setValues(self)
            '''
            x = 0
            if(self.y > 50):
                animationUpper = (h + self.y*2 - 140)/50
                animationDown = (h)/50
            elif(self.y < 50):
                animationUpper = (h)/50
                animationDown = (140-(h + self.y))/50
            elif(self.y == 50):
                animationUpper = (h)/50
                animationDown = (h)/50
            '''

            time.sleep(0.3)
            # saveup = self.upper
            # savedown = self.down

            saveup_right = self.upper_right
            saveup_left = self.upper_left
            savedown_right = self.down_right
            savedown_left = self.down_left

            #while(x<50):
            #self.upper = 0 #self.upper - animationUpper
            #self.down = 0 #self.down - animationDown
            self.output.data = [0, 0, 0, 0]
                #print("subtrai: "+str(self.upper))
                #time.sleep(0.008)
                #x = x + 1
            #x = 0
            #while(x<50):
            time.sleep(0.3)
            #self.upper = saveup #self.upper + animationUpper
            #self.down = savedown #self.down + animationDown
            self.output.data = [saveup_right, saveup_left, savedown_right, savedown_left]
                #print("soma: "+str(self.upper))
                #time.sleep(0.008)
                #x = x + 1
            self.animation = 0
            time.sleep(frequency)  
            

if __name__ == '__main__':
    try:
        eyelidEnable()
    except rospy.ROSInterruptException:
        pass