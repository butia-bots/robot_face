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
    "doubt":5,
    "sleepy":6
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
            self.rightY = self.rightY_standard_params                   # 
            self.leftY = self.leftY_standard_params                     # 
            self.rightRotation = self.rightRotation_standard_params     #      
            self.leftRotation = self.leftRotation_standard_params       # 
        
        elif(self.emotion == EMOTIONS["happy"]):                        # Happy
            self.rightY = self.rightY_happy_params                      # 
            self.leftY = self.leftY_happy_params                        # 
            self.rightRotation = self.rightRotation_happy_params        # 
            self.leftRotation = self.leftRotation_happy_params          # 
        
        elif(self.emotion == EMOTIONS["sad"]):                          # Sad
            self.rightY = self.rightY_sad_params                        # 
            self.leftY = self.leftY_sad_params                          # 
            self.rightRotation = self.rightRotation_sad_params          # 
            self.leftRotation = self.leftRotation_sad_params            # 
       
        elif(self.emotion == EMOTIONS["rage"]):                         # Rage
            self.rightY = self.rightY_rage_params                       # 
            self.leftY = self.leftY_rage_params                         # 
            self.rightRotation = self.rightRotation_rage_params         # 
            self.leftRotation = self.leftRotation_rage_params           # 
       
        elif(self.emotion == EMOTIONS["scared"]):                       # Scared
            self.rightY = self.rightY_scared_params                     # 
            self.leftY = self.leftY_scared_params                       # 
            self.rightRotation = self.rightRotation_scared_params       # 
            self.leftRotation = self.leftRotation_scared_params         # 

        elif(self.emotion == EMOTIONS["doubt"]):                       # doubt
            self.rightY = self.rightY_doubt_params                     # 
            self.leftY = self.leftY_doubt_params                       # 
            self.rightRotation = self.rightRotation_doubt_params       # 
            self.leftRotation = self.leftRotation_doubt_params         # 

        elif(self.emotion == EMOTIONS["sleepy"]):                       # sleepy
            self.rightY = self.rightY_sleepy_params                     # 
            self.leftY = self.leftY_sleepy_params                       # 
            self.rightRotation = self.rightRotation_sleepy_params       # 
            self.leftRotation = self.leftRotation_sleepy_params         # 

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

        self.rightY_doubt_params = rospy.get_param("butia_emotions/eyebrown/doubt/rightY")
        self.leftY_doubt_params = rospy.get_param("butia_emotions/eyebrown/doubt/leftY")
        self.rightRotation_doubt_params = rospy.get_param("butia_emotions/eyebrown/doubt/rightRotation")
        self.leftRotation_doubt_params = rospy.get_param("butia_emotions/eyebrown/doubt/leftRotation")

        self.rightY_sleepy_params = rospy.get_param("butia_emotions/eyebrown/sleepy/rightY")
        self.leftY_sleepy_params = rospy.get_param("butia_emotions/eyebrown/sleepy/leftY")
        self.rightRotation_sleepy_params = rospy.get_param("butia_emotions/eyebrown/sleepy/rightRotation")
        self.leftRotation_sleepy_params = rospy.get_param("butia_emotions/eyebrown/sleepy/leftRotation")

if __name__ == '__main__':
    try:
        eyebrownEnable()
    except rospy.ROSInterruptException:
        pass