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

        self.percent = self.percent_standard_params

        while not rospy.is_shutdown():
            self.getOutput()
            self.output.data = []
            self.output.data = [self.percent]
            pub.publish(self.output)
            rate.sleep()
        
    def getOutput(self):

        if(self.emotion == EMOTIONS["standard"]):
            self.percent = self.percent_standard_params

        elif(self.emotion == EMOTIONS["happy"]):
            self.percent = self.percent_happy_params

        elif(self.emotion == EMOTIONS["sad"]):   
            self.percent = self.percent_sad_params
            
        elif(self.emotion == EMOTIONS["rage"]):
            self.percent = self.percent_rage_params

        elif(self.emotion == EMOTIONS["scared"]):  
            self.percent = self.percent_scared_params

        elif(self.emotion == EMOTIONS["doubt"]):  
            self.percent = self.percent_doubt_params

        elif(self.emotion == EMOTIONS["sleepy"]):  
            self.percent = self.percent_sleepy_params

    def getJaw_st(self, msg):
        self.data = msg.data
        self.emotion=self.data

    def _readParameters(self):
        self.percent_standard_params = rospy.get_param("butia_emotions/jaw/standard/percent")

        self.percent_happy_params = rospy.get_param("butia_emotions/jaw/happy/percent")

        self.percent_sad_params = rospy.get_param("butia_emotions/jaw/sad/percent")

        self.percent_rage_params = rospy.get_param("butia_emotions/jaw/rage/percent")

        self.percent_scared_params = rospy.get_param("butia_emotions/jaw/scared/percent")

        self.percent_doubt_params = rospy.get_param("butia_emotions/jaw/doubt/percent")

        self.percent_sleepy_params = rospy.get_param("butia_emotions/jaw/sleepy/percent")


if __name__ == '__main__':
    try:
        jawEnable()
    except rospy.ROSInterruptException:
        pass