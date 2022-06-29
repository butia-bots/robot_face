#!/usr/bin/env python3

import rospy
import time
import threading
from std_msgs.msg import (Int16MultiArray, Int16)
from PyDynamixel import DxlComm, Joint

MOTORS_IDX = {
    "EyebrowRightHeight": 0,
    "EyebrowLeftHeight": 1,
    "EyebrowRightAngle": 2,
    "EyebrowLeftAngle": 3,
    "EyelidRightUp": 4,
    "EyelidLeftUp": 5,
    "EyelidRightDown": 6,
    "EyelidLeftDown": 7,
    "EyeHorizontal": 8,
    "EyeVertical": 9,
    "Mouth": 10,
}

class dataflowEnable():
    def __init__(self):
        rospy.init_node('dataController', anonymous=False)
        rate = rospy.Rate(100) # 100hz

        # Define the output vector
        self.motors = [50] * 13

        self.port = DxlComm(commPort="/dev/ttyACM1")
        self.joint = Joint(128)
        self.port.attachJoint(self.joint)

        self.sub_mouth = Int16MultiArray()
        self.sub_mouth.data = []  
        self.sub_mouth = rospy.Subscriber('mouth', Int16MultiArray, self.getMouth)

        self.sub_eye = Int16MultiArray()
        self.sub_eye.data = []  
        self.sub_eye = rospy.Subscriber('eye', Int16MultiArray, self.getEye)

        self.sub_eyelid = Int16MultiArray()
        self.sub_eyelid.data = []  
        self.sub_eyelid = rospy.Subscriber('eyelid', Int16MultiArray, self.getEyelid)

        self.sub_eyebrown = Int16MultiArray()
        self.sub_eyebrown.data = []  
        self.sub_eyebrown = rospy.Subscriber('eyebrown', Int16MultiArray, self.getEyebrown)

        #self.sub_neck = Int16MultiArray()
        #self.sub_neck.data = []  
        #self.sub_neck = rospy.Subscriber('neck', Int16MultiArray, self.getNeck)

        # updateLoop = threading.Thread(name = 'send2Arduino', target = dataflowEnable.sendArduino, args = (self,))
        # updateLoop.setDaemon(True)
        # updateLoop.start()

        while not rospy.is_shutdown():
            # 0 - EyebrowRightHeight
            # 1 - EyebrowLeftHeight
            # 2 - EyebrowRightAngle
            # 3 - EyebrowLeftAngle
            # 4 - EyelidRightUp
            # 5 - EyelidLeftUp
            # 6 - EyelidRightDown
            # 7 - EyelidLeftDown
            # 8 - EyeHorizontal
            # 9 - EyeVertical
            # 10 - Mouth

            # print (self.motors)
            
            self.joint.writeValue(4, int(self.motors[MOTORS_IDX["EyelidRightUp"]]))
            self.joint.writeValue(5, int(self.motors[MOTORS_IDX["EyelidLeftUp"]]))
            self.joint.writeValue(6, int(self.motors[MOTORS_IDX["EyelidRightDown"]]))
            self.joint.writeValue(7, int(self.motors[MOTORS_IDX["EyelidLeftDown"]]))
            self.joint.writeValue(10, int(self.motors[MOTORS_IDX["Mouth"]]))
            self.joint.writeValue(0, int(self.motors[MOTORS_IDX["EyebrowRightHeight"]]))
            self.joint.writeValue(1, int(self.motors[MOTORS_IDX["EyebrowLeftHeight"]]))
            self.joint.writeValue(2, int(self.motors[MOTORS_IDX["EyebrowRightAngle"]]))
            self.joint.writeValue(3, int(self.motors[MOTORS_IDX["EyebrowLeftAngle"]]))
            self.joint.writeValue(8, int(self.motors[MOTORS_IDX["EyeHorizontal"]]))
            self.joint.writeValue(9, int(self.motors[MOTORS_IDX["EyeVertical"]]))
            rate.sleep()

    def getMouth(self, msg):
        data = msg.data
        #self.motors[0] = int(0.3059*self.data[0])
        #self.motors[10] = abs(100-data[0])
        self.motors[MOTORS_IDX["Mouth"]] = data[1]
        #self.motors[1] = data[1]

    def getEye(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyeHorizontal"]] = data[0]
        self.motors[MOTORS_IDX["EyeVertical"]] = data[1]
    
    def getEyelid(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyelidRightUp"]] = data[0]
        self.motors[MOTORS_IDX["EyelidLeftUp"]] = data[1]
        self.motors[MOTORS_IDX["EyelidRightDown"]] = data[2]
        self.motors[MOTORS_IDX["EyelidLeftDown"]] = data[3]
        self.motors[11] = data[2]
        self.motors[12] = data[0]

    def getEyebrown(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyebrowRightHeight"]] = data[0]
        self.motors[MOTORS_IDX["EyebrowLeftHeight"]] = data[1]
        self.motors[MOTORS_IDX["EyebrowRightAngle"]] = data[2]
        self.motors[MOTORS_IDX["EyebrowLeftAngle"]] = data[3]

    def getNeck(self, msg):
       data = msg.data
       self.motors[12] = data[0]
       self.motors[13] = data[1]

    def sendArduino(self):
        while(True):
            # 0 - EyebrowRightHeight
            # 1 - EyebrowLeftHeight
            # 2 - EyebrowRightAngle
            # 3 - EyebrowLeftAngle
            # 4 - EyelidRightUp
            # 5 - EyelidLeftUp
            # 6 - EyelidRightDown
            # 7 - EyelidLeftDown
            # 8 - EyeHorizontal
            # 9 - EyeVertical
            # 10 - Mouth

            print (self.motors)

            
            self.joint.writeValue(4, int(self.motors[MOTORS_IDX["EyelidRightUp"]]))
            self.joint.writeValue(5, int(self.motors[MOTORS_IDX["EyelidLeftUp"]]))
            self.joint.writeValue(6, int(self.motors[MOTORS_IDX["EyelidRightDown"]]))
            self.joint.writeValue(7, int(self.motors[MOTORS_IDX["EyelidLeftDown"]]))
            #self.joint.writeValue(10, int(self.motors[MOTORS_IDX["Mouth"]]))
            self.joint.writeValue(0, int(self.motors[MOTORS_IDX["EyebrowRightHeight"]]))
            self.joint.writeValue(1, int(self.motors[MOTORS_IDX["EyebrowLeftHeight"]]))
            self.joint.writeValue(2, int(self.motors[MOTORS_IDX["EyebrowRightAngle"]]))
            self.joint.writeValue(3, int(self.motors[MOTORS_IDX["EyebrowLeftAngle"]]))
            self.joint.writeValue(8, int(self.motors[MOTORS_IDX["EyeHorizontal"]]))
            self.joint.writeValue(9, int(self.motors[MOTORS_IDX["EyeVertical"]]))
            
            time.sleep(0.001)

if __name__ == '__main__':
    try:
        dataflowEnable()
    except rospy.ROSInterruptException:
        pass