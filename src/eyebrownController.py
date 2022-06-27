#!/usr/bin/env python3

import rospy
from std_msgs.msg import (Int16, Int16MultiArray)

# Define output vector
output = Int16MultiArray()
output.data = []

class eyebrownEnable():
    def __init__(self):
        pub = rospy.Publisher('eyebrown', Int16MultiArray, queue_size=10)
        rospy.init_node('eyebrownEnable', anonymous=False)
        self.sub_eyebrown_st = rospy.Subscriber('emotion', Int16, self.getEyebrown_st)
        rate = rospy.Rate(50) # 50hz

        # Set normal emotion on eyebrown
        self.emotion = 0
        self.rightY = 20
        self.leftY = 20
        self.rightRotation = 50
        self.leftRotation = 50

        while not rospy.is_shutdown():
            eyebrownEnable.getOutput(self)
            output.data = []
            output.data = [self.rightY, self.leftY, self.rightRotation, self.leftRotation]
            pub.publish(output)
            rate.sleep()
        
    def getOutput(self):
        if(self.emotion == 0): #Poker face | OK
            self.rightY = 65
            self.leftY = 50
            self.rightRotation = 85
            self.leftRotation = 130
        elif(self.emotion == 1): #Happy | Ok
            self.rightY = 130
            self.leftY = 105
            self.rightRotation = 80
            self.leftRotation = 125
        elif(self.emotion == 2): #Sad | Ok
            self.rightY = 65
            self.leftY = 50
            self.rightRotation = 45
            self.leftRotation = 20
        elif(self.emotion == 3): #Rage
            self.rightY = 20
            self.leftY = 20
            self.rightRotation = 140
            self.leftRotation = 160
        elif(self.emotion == 4): #Scared
            self.rightY = 130
            self.leftY = 105
            self.rightRotation = 75
            self.leftRotation = 120

    def getEyebrown_st(self, msg):
        self.data = msg.data
        self.emotion = self.data

if __name__ == '__main__':
    try:
        eyebrownEnable()
    except rospy.ROSInterruptException:
        pass