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
        rate = rospy.Rate(50)

        self.emotion = 0
        self.rightY = 50                # 20
        self.leftY = 50                 # 20
        self.rightRotation = 50
        self.leftRotation = 50

        while not rospy.is_shutdown():
            eyebrownEnable.getOutput(self)
            output.data = []
            output.data = [self.rightY, self.leftY, self.rightRotation, self.leftRotation]
            pub.publish(output)
            rate.sleep()
        
    def getOutput(self):
        if(self.emotion == 0):          # Poker Face
            self.rightY = 50            # 65
            self.leftY = 50             # 50
            self.rightRotation = 50     # 85     
            self.leftRotation = 50      # 130
        elif(self.emotion == 1):        # Happy
            self.rightY = 90            # 130
            self.leftY = 90             # 105
            self.rightRotation = 60     # 80
            self.leftRotation = 60      # 125
        elif(self.emotion == 2):        # Sad
            self.rightY = 30            # 65
            self.leftY = 30             # 50
            self.rightRotation = 20     # 45
            self.leftRotation = 20      # 20
        elif(self.emotion == 3):        # Rage
            self.rightY = 10            # 20
            self.leftY = 10             # 20
            self.rightRotation = 80     # 140
            self.leftRotation = 80      # 160
        elif(self.emotion == 4):        # Scared
            self.rightY = 100           # 130
            self.leftY = 100            # 105
            self.rightRotation = 70     # 75
            self.leftRotation = 70      # 120

    def getEyebrown_st(self, msg):
        self.data = msg.data
        self.emotion = self.data

if __name__ == '__main__':
    try:
        eyebrownEnable()
    except rospy.ROSInterruptException:
        pass