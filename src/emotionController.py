#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int16

class emotionEnable():
    def __init__(self):
        pub = rospy.Publisher('emotion', Int16, queue_size=10)
        rospy.init_node('emotionEnable', anonymous=False)
        self.sub_eyelid_st = rospy.Subscriber('updateEmotion', Int16, self.getEmotion)
        rate = rospy.Rate(50) 
        self.emotion = 0

        while not rospy.is_shutdown():
            pub.publish(self.emotion)
            rate.sleep()

    def getEmotion(self, msg):
        self.data = msg.data
        self.emotion = self.data

if __name__ == '__main__':
    try:
        emotionEnable()
    except rospy.ROSInterruptException:
        pass