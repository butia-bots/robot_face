#!/usr/bin/env python3

from __future__ import division
import rospy
import time
import threading
from std_msgs.msg import (Int16MultiArray, Int16)
import map as mp


EMOTIONS = {
    "standard": 0,
    "happy": 1,
    "sad": 2,
    "rage": 3,
    "scared": 4,
    "doubt":5,
    "sleepy":6
}

class jawEnable():
    def __init__(self):
        rospy.init_node('jawEnable', anonymous=False)
        pub = rospy.Publisher('mouth', Int16MultiArray, queue_size=10)

        self.sub_jaw_st = rospy.Subscriber('emotion', Int16, self.getJaw_st)
        
        rate = rospy.Rate(30)

        # Define the output vector
        self.output = Int16MultiArray()
        self.output.data = []

        self.emotion = 0

        self._readParameters()

        self.xPosition = self.left_standard_params
        self.yPosition = self.right_standard_params

        while not rospy.is_shutdown():
            self.getOutput()
            self.output.data = []
            self.output.data = [self.xPosition , self.yPosition]
            pub.publish(self.output)
            rate.sleep()
        
    def getOutput(self):

        if(self.emotion == EMOTIONS["standard"]):
            self.xPosition = self.left_standard_params                   # 
            self.yPosition = self.right_standard_params  

        elif(self.emotion == EMOTIONS["happy"]):
            self.xPosition = self.left_happy_params                   # 
            self.yPosition = self.right_happy_params 

        elif(self.emotion == EMOTIONS["sad"]):   
            self.xPosition = self.left_sad_params                   # 
            self.yPosition = self.right_sad_params 
            
        elif(self.emotion == EMOTIONS["rage"]):
            self.xPosition = self.left_rage_params                   # 
            self.yPosition = self.right_rage_params 

        elif(self.emotion == EMOTIONS["scared"]):  
            self.xPosition = self.left_scared_params                   # 
            self.yPosition = self.right_scared_params

        elif(self.emotion == EMOTIONS["doubt"]):  
            self.xPosition = self.left_doubt_params                   # 
            self.yPosition = self.right_doubt_params

        elif(self.emotion == EMOTIONS["sleepy"]):  
            self.xPosition = self.left_sleepy_params                   # 
            self.yPosition = self.right_sleepy_params

    def getJaw_st(self, msg):
        self.data = msg.data
        self.emotion=self.data

    def _readParameters(self):
        self.left_standard_params = rospy.get_param("butia_emotions/jaw/standard/left")
        self.right_standard_params = rospy.get_param("butia_emotions/jaw/standard/right")

        self.left_happy_params = rospy.get_param("butia_emotions/jaw/happy/left")
        self.right_happy_params = rospy.get_param("butia_emotions/jaw/happy/right")

        self.left_sad_params = rospy.get_param("butia_emotions/jaw/sad/left")
        self.right_sad_params = rospy.get_param("butia_emotions/jaw/sad/right")

        self.left_rage_params = rospy.get_param("butia_emotions/jaw/rage/left")
        self.right_rage_params = rospy.get_param("butia_emotions/jaw/rage/right")

        self.left_scared_params = rospy.get_param("butia_emotions/jaw/scared/left")
        self.right_scared_params = rospy.get_param("butia_emotions/jaw/scared/right")

        self.left_doubt_params = rospy.get_param("butia_emotions/jaw/doubt/left")
        self.right_doubt_params = rospy.get_param("butia_emotions/jaw/doubt/right")

        self.left_sleepy_params = rospy.get_param("butia_emotions/jaw/sleepy/left")
        self.right_sleepy_params = rospy.get_param("butia_emotions/jaw/sleepy/right")


if __name__ == '__main__':
    try:
        jawEnable()
    except rospy.ROSInterruptException:
        pass