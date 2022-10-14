#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from std_msgs.msg import Float64MultiArray, Int16

EMOTIONS = {
    "standard": 0,
    "happy": 1,
    "sad": 2,
    "rage": 3,
    "scared": 4,
}

neckPub = None

horizontal_standard_params = 0
vertical_standard_params = 0

horizontal_happy_params = 0
vertical_happy_params = 0

horizontal_sad_params = 0
vertical_sad_params = 0
    
horizontal_rage_params = 0
vertical_rage_params = 0
    
horizontal_scared_params = 0
vertical_scared_params = 0

def _readParameters():
    global horizontal_standard_params
    global vertical_standard_params

    global horizontal_happy_params
    global vertical_happy_params

    global horizontal_sad_params
    global vertical_sad_params
    
    global horizontal_rage_params
    global vertical_rage_params
    
    global horizontal_scared_params
    global vertical_scared_params

    horizontal_standard_params = rospy.get_param("butia_emotions/neck/standard/horizontal")
    vertical_standard_params = rospy.get_param("butia_emotions/neck/standard/vertical")

    horizontal_happy_params = rospy.get_param("butia_emotions/neck/happy/horizontal")
    vertical_happy_params = rospy.get_param("butia_emotions/neck/happy/vertical")

    horizontal_sad_params = rospy.get_param("butia_emotions/neck/sad/horizontal")
    vertical_sad_params = rospy.get_param("butia_emotions/neck/sad/vertical")

    horizontal_rage_params = rospy.get_param("butia_emotions/neck/rage/horizontal")
    vertical_rage_params = rospy.get_param("butia_emotions/neck/rage/vertical")

    horizontal_scared_params = rospy.get_param("butia_emotions/neck/scared/horizontal")
    vertical_scared_params = rospy.get_param("butia_emotions/neck/scared/vertical")

def get(msg):
    neckData = Float64MultiArray()

    horizontal = 180
    vertical = 0

    data = msg.data

    if(data == EMOTIONS["standard"]):
        horizontal = horizontal_standard_params
        vertical = vertical_standard_params
    elif(data == EMOTIONS["happy"]):
        horizontal = horizontal_happy_params
        vertical = vertical_happy_params
    elif(data == EMOTIONS["sad"]):
        horizontal = horizontal_sad_params
        vertical = vertical_sad_params
    elif(data == EMOTIONS["rage"]):
        horizontal = horizontal_rage_params
        vertical = vertical_rage_params
    elif(data == EMOTIONS["scared"]):
        horizontal = horizontal_scared_params
        vertical = vertical_scared_params

    neckData.data = [float(horizontal), float(vertical)]
    neckPub.publish(neckData)

def set_initial_position():
    horizontal = 180
    vertical = 0

    neckData = Float64MultiArray()

    neckData.data = [float(horizontal), float(vertical)]
    neckPub.publish(neckData)

if __name__ == '__main__':
    rospy.init_node('neckController', anonymous=False)    
    neckPub = rospy.Publisher("neck", Float64MultiArray, queue_size = 10)
    # _readParameters()

    set_initial_position()

