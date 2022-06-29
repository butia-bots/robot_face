#!/usr/bin/env python3

import rospy
from std_msgs.msg import (Int16, Int16MultiArray)

# Define output vector
output = Int16MultiArray()
output.data = []

EMOTIONS = {
    "standard": 0,
    "happy": 1,
    "sad": 2,
    "rage": 3,
    "scared": 4,
}

class eyebrownEnable():
    def __init__(self):
        pub = rospy.Publisher('eyebrown', Int16MultiArray, queue_size=10)
        rospy.init_node('eyebrownEnable', anonymous=False)
        self.sub_eyebrown_st = rospy.Subscriber('emotion', Int16, self.getEyebrown_st)
        rate = rospy.Rate(50)

        self.emotion = 0

        self._readParameters()

        self.rightY = self.rightY_standard_params
        self.leftY = self.leftY_standard_params
        self.rightRotation = self.rightRotation_standard_params
        self.leftRotation = self.leftRotation_standard_params

        while not rospy.is_shutdown():
            eyebrownEnable.getOutput(self)
            output.data = []
            output.data = [self.rightY, self.leftY, self.rightRotation, self.leftRotation]
            pub.publish(output)
            rate.sleep()
        
    def getOutput(self):
        if(self.emotion == EMOTIONS["standard"]):                       # Standard
            self.rightY = self.rightY_standard_params                   # 65
            self.leftY = self.leftY_standard_params                     # 50
            self.rightRotation = self.rightRotation_standard_params     # 85     
            self.leftRotation = self.leftRotation_standard_params       # 130
        elif(self.emotion == EMOTIONS["happy"]):                        # Happy
            self.rightY = self.rightY_happy_params                      # 130
            self.leftY = self.leftY_happy_params                        # 105
            self.rightRotation = self.rightRotation_happy_params        # 80
            self.leftRotation = self.leftRotation_happy_params          # 125
        elif(self.emotion == EMOTIONS["sad"]):                          # Sad
            self.rightY = self.rightY_sad_params                        # 65
            self.leftY = self.leftY_sad_params                          # 50
            self.rightRotation = self.rightRotation_sad_params          # 45
            self.leftRotation = self.leftRotation_sad_params            # 20
        elif(self.emotion == EMOTIONS["rage"]):                         # Rage
            self.rightY = self.rightY_rage_params                       # 20
            self.leftY = self.leftY_rage_params                         # 20
            self.rightRotation = self.rightRotation_rage_params         # 140
            self.leftRotation = self.leftRotation_rage_params           # 160
        elif(self.emotion == EMOTIONS["scared"]):                       # Scared
            self.rightY = self.rightY_scared_params                     # 130
            self.leftY = self.leftY_scared_params                       # 105
            self.rightRotation = self.rightRotation_scared_params       # 75
            self.leftRotation = self.leftRotation_scared_params         # 120

    def getEyebrown_st(self, msg):
        self.data = msg.data
        self.emotion = self.data

    def _readParameters(self):
        self.rightY_standard_params = rospy.get_param("butia_emotions/eyebrown/standard/rightY")
        self.leftY_standard_params = rospy.get_param("butia_emotions/eyebrown/standard/leftY")
        self.rightRotation_standard_params = rospy.get_param("butia_emotions/eyebrown/standard/rightRotation")
        self.leftRotation_standard_params = rospy.get_param("butia_emotions/eyebrown/standard/leftRotation")

        self.rightY_happy_params = rospy.get_param("butia_emotions/eyebrown/happy/rightY")
        self.leftY_happy_params = rospy.get_param("butia_emotions/eyebrown/happy/leftY")
        self.rightRotation_happy_params = rospy.get_param("butia_emotions/eyebrown/happy/rightRotation")
        self.leftRotation_happy_params = rospy.get_param("butia_emotions/eyebrown/happy/leftRotation")

        self.rightY_sad_params = rospy.get_param("butia_emotions/eyebrown/sad/rightY")
        self.leftY_sad_params = rospy.get_param("butia_emotions/eyebrown/sad/leftY")
        self.rightRotation_sad_params = rospy.get_param("butia_emotions/eyebrown/sad/rightRotation")
        self.leftRotation_sad_params = rospy.get_param("butia_emotions/eyebrown/sad/leftRotation")

        self.rightY_rage_params = rospy.get_param("butia_emotions/eyebrown/rage/rightY")
        self.leftY_rage_params = rospy.get_param("butia_emotions/eyebrown/rage/leftY")
        self.rightRotation_rage_params = rospy.get_param("butia_emotions/eyebrown/rage/rightRotation")
        self.leftRotation_rage_params = rospy.get_param("butia_emotions/eyebrown/rage/leftRotation")

        self.rightY_scared_params = rospy.get_param("butia_emotions/eyebrown/scared/rightY")
        self.leftY_scared_params = rospy.get_param("butia_emotions/eyebrown/scared/leftY")
        self.rightRotation_scared_params = rospy.get_param("butia_emotions/eyebrown/scared/rightRotation")
        self.leftRotation_scared_params = rospy.get_param("butia_emotions/eyebrown/scared/leftRotation")

if __name__ == '__main__':
    try:
        eyebrownEnable()
    except rospy.ROSInterruptException:
        pass