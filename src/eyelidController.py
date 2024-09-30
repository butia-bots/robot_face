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

EMOTIONS = {
    "standard": 0,
    "happy": 1,
    "sad": 2,
    "rage": 3,
    "scared": 4,
    "doubt":5,
    "sleepy":6
}

class eyelidEnable():
    def __init__(self):
        rospy.init_node('eyelidEnable', anonymous=False)
        self.pub = rospy.Publisher('eyelid', Int16MultiArray, queue_size=10)

        self.sub_eyelid_st = rospy.Subscriber('emotion', Int16, self.getEyelid_st)
        # self.sub_eyelid_dn = Int16MultiArray()
        # self.sub_eyelid_dn.data = []   
        # self.sub_eyelid_dn = rospy.Subscriber('eye', Int16MultiArray, self.getEyelid_dn)
        
        rate = rospy.Rate(30)

        # Define the output vector
        self.output = Int16MultiArray()
        self.output.data = []

        self.animation = 0

        # self.y = 50 
        # self.upper = 0
        # self.down = 0

        self.emotion = 0

        self._readParameters()

        self.right = self.right_standard_params
        self.left = self.left_standard_params

        # Start blink thread
        '''blinkLoop = threading.Thread(name = 'blink', target = eyelidEnable.blink, args = (self,))
        blinkLoop.setDaemon(True)
        blinkLoop.start()'''
        
        self.blinking = 0
      #  blink_timer = rospy.Timer(rospy.Duration(secs=10), self.blink)


        while not rospy.is_shutdown():
            self.getOutput()
            self.output.data = []
            self.output.data = [self.right, self.left]
            self.pub.publish(self.output)
            rate.sleep()
        
    def getOutput(self):
        # if (self.animation == 1):
        #     pass
        # elif(self.animation == 0): 
        #     self.setValues()

        if(self.emotion == EMOTIONS["standard"]):
            self.right = self.right_standard_params
            self.left = self.left_standard_params

        elif(self.emotion == EMOTIONS["happy"]):
            self.right = self.right_happy_params
            self.left = self.left_happy_params

        elif(self.emotion == EMOTIONS["sad"]):   
            self.right = self.right_sad_params
            self.left = self.left_sad_params

        elif(self.emotion == EMOTIONS["rage"]):
            self.right = self.right_rage_params
            self.left = self.left_rage_params

        elif(self.emotion == EMOTIONS["scared"]):  
            self.right = self.right_scared_params
            self.left = self.left_scared_params

        elif(self.emotion == EMOTIONS["doubt"]):  
            self.right = self.right_doubt_params
            self.left = self.left_doubt_params

        elif(self.emotion == EMOTIONS["sleepy"]):  
            self.right = self.right_sleepy_params
            self.left = self.left_sleepy_params
    
    def setValues(self):
        pass
        '''
        if(self.y > 50):
            self.upper = (h + self.y*2 - 140)
            self.down = 140
            self.output.data = [self.upper, self.upper, self.down, self.down]
        elif(self.y < 50):
            self.upper = h
            self.down = 140 - (h + self.y)
            self.output.data = [self.upper, self.upper, self.down, self.down]
        elif(self.y == 50):
            self.upper = 100-h
            self.down = 100-h
            self.output.data = [self.upper, self.upper, self.down, self.down]
        '''

    def getEyelid_dn(self, msg):
        self.data = msg.data
        self.y = 50

    def getEyelid_st(self, msg):
        global h
        global frequency

        self.data = msg.data
        self.emotion=self.data

        self.getOutput()
        self.output.data = []
        self.output.data = [self.right, self.left]
        self.pub.publish(self.output)

        '''
        if(self.data == EMOTIONS["standard"]):
            h = self.h_standard_params
            frequency = 0
        elif(self.data == EMOTIONS["happy"]):
            h = self.h_happy_params 
            frequency = 0
        elif(self.data == EMOTIONS["sad"]):   
            h = self.h_sad_params
            frequency = 0
        elif(self.data == EMOTIONS["rage"]):
            h = self.h_rage_params
            frequency = 0
        elif(self.data == EMOTIONS["scared"]):  
            h = self.h_scared_params
            frequency = 0
        elif(self.data == EMOTIONS["doubt"]):  
            h = self.h_doubt_params
            frequency = 0
        elif(self.data == EMOTIONS["sleepy"]):  
            h = self.h_sleepy_params
            frequency = 0
        '''
    def _readParameters(self):
        self.right_standard_params = rospy.get_param("butia_emotions/eyelid/standard/right")
        self.left_standard_params = rospy.get_param("butia_emotions/eyelid/standard/left")

        self.right_happy_params = rospy.get_param("butia_emotions/eyelid/happy/right")
        self.left_happy_params = rospy.get_param("butia_emotions/eyelid/happy/left")

        self.right_sad_params = rospy.get_param("butia_emotions/eyelid/sad/right")
        self.left_sad_params = rospy.get_param("butia_emotions/eyelid/sad/left")

        self.right_rage_params = rospy.get_param("butia_emotions/eyelid/rage/right")
        self.left_rage_params = rospy.get_param("butia_emotions/eyelid/rage/left")

        self.right_scared_params = rospy.get_param("butia_emotions/eyelid/scared/right")
        self.left_scared_params = rospy.get_param("butia_emotions/eyelid/scared/left")

        self.right_doubt_params = rospy.get_param("butia_emotions/eyelid/doubt/right")
        self.left_doubt_params = rospy.get_param("butia_emotions/eyelid/doubt/left")

        self.right_sleepy_params = rospy.get_param("butia_emotions/eyelid/sleepy/right")
        self.left_sleepy_params = rospy.get_param("butia_emotions/eyelid/sleepy/left")

        '''self.h_standard_params = rospy.get_param("butia_emotions/eyelid/standard/h")
        self.frequency_standard_params = rospy.get_param("butia_emotions/eyelid/standard/frequency")

        self.h_happy_params = rospy.get_param("butia_emotions/eyelid/happy/h")
        self.frequency_happy_params = rospy.get_param("butia_emotions/eyelid/happy/frequency")

        self.h_sad_params = rospy.get_param("butia_emotions/eyelid/sad/h")
        self.frequency_sad_params = rospy.get_param("butia_emotions/eyelid/sad/frequency")

        self.h_rage_params = rospy.get_param("butia_emotions/eyelid/rage/h")
        self.frequency_rage_params = rospy.get_param("butia_emotions/eyelid/rage/frequency")

        self.h_scared_params = rospy.get_param("butia_emotions/eyelid/scared/h")
        self.frequency_scared_params = rospy.get_param("butia_emotions/eyelid/scared/frequency")

        self.h_doubt_params = rospy.get_param("butia_emotions/eyelid/doubt/h")
        self.frequency_doubt_params = rospy.get_param("butia_emotions/eyelid/doubt/frequency")

        self.h_sleepy_params = rospy.get_param("butia_emotions/eyelid/sleepy/h")
        self.frequency_sleepy_params = rospy.get_param("butia_emotions/eyelid/sleepy/frequency")'''
    
    def blink(self):
        

        self.output.data = []
        
        if self.blinking:
            self.output.data = [self.right, self.left]
            self.blinking=0

        else:
            self.output.data = [0, 0]
            self.blinking=1
            endblinking = rospy.Timer(rospy.Duration(nsecs=150000000), self.blink, oneshot=True)

        self.pub.publish(self.output)


        pass
        '''
        while(True):
            time.sleep(0.3)
            self.output.data = [100, 100, 100, 100]
            self.animation = 1
            time.sleep(0.3)
            self.setValues()
            self.animation = 0
            
            time.sleep(frequency)  
        '''
            

if __name__ == '__main__':
    try:
        eyelidEnable()
    except rospy.ROSInterruptException:
        pass