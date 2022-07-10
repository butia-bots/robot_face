#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray, Bool, Float64MultiArray
from PyDynamixel import DxlCommProtocol1, DxlCommProtocol2, JointProtocol1, JointProtocol2

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
    "NeckHorizontal": 11,
    "NeckVertical": 12,
}

class dataflowEnable():
    def __init__(self, pause=False):
        self.pause = pause

        rospy.init_node('dataController', anonymous=False)
        rate = rospy.Rate(100) # 100hz

        try:
            # If use Dynamixel Protocol 1, uncomment the next line
            # self.neck_port = DxlCommProtocol2("/dev/ttyUSB0") 

            # If use Dynamixel Protocol 2, uncomment the next line
            self.neck_port = DxlCommProtocol2("/dev/ttyUSB0")

            # If use Dynamixel Protocol 1, uncomment the next two lines
            # self.neckHorizontal = JointProtocol1(62)
            # self.neckVertical = JointProtocol1(61)

            # If use Dynamixel Protocol 2, uncomment the next two lines
            self.neckHorizontal = JointProtocol2(62)
            self.neckVertical = JointProtocol2(61)

            self.neck_port.attachJoint(self.neckVertical)
            self.neck_port.attachJoint(self.neckHorizontal)

            # Ativa o torque dos motores, por seguranca
            self.neckHorizontal.enableTorque()
            self.neckVertical.enableTorque()

            self.neckHorizontal.setVelocityLimit(limit=80)
            self.neckVertical.setVelocityLimit(limit=40)
        except:
            pass

        # Define the output vector
        self.motors = [50] * 13

        self.motors[MOTORS_IDX["EyebrowRightHeight"]] = 20
        self.motors[MOTORS_IDX["EyebrowLeftHeight"]] = 20
        self.motors[MOTORS_IDX["EyebrowRightAngle"]] = 50
        self.motors[MOTORS_IDX["EyebrowLeftAngle"]] = 50
        self.motors[MOTORS_IDX["EyelidRightUp"]] = 20
        self.motors[MOTORS_IDX["EyelidLeftUp"]] = 20
        self.motors[MOTORS_IDX["EyelidRightDown"]] = 20
        self.motors[MOTORS_IDX["EyelidLeftDown"]] = 20
        self.motors[MOTORS_IDX["EyeHorizontal"]] = 40
        self.motors[MOTORS_IDX["EyeVertical"]] = 85
        self.motors[MOTORS_IDX["Mouth"]] = 100

        self.port = DxlCommProtocol1(commPort="/dev/ttyACM0")
        self.joint = JointProtocol1(128)
        self.port.attachJoint(self.joint)

        rospy.Subscriber("/RosAria/motors_state", Bool, self.setPause)

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

        self.sub_neck = Float64MultiArray()
        self.sub_neck.data = []  
        self.sub_neck = rospy.Subscriber('neck', Float64MultiArray, self.getNeck)

        # updateLoop = threading.Thread(name = 'send2Arduino', target = dataflowEnable.sendArduino, args = (self,))
        # updateLoop.setDaemon(True)
        # updateLoop.start()

        while not rospy.is_shutdown():
            if not self.pause: 
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
                # self.neckHorizontal.sendGoalAngle(self.motors[MOTORS_IDX["NeckHorizontal"]])
                # self.neckVertical.sendGoalAngle(self.motors[MOTORS_IDX["NeckVertical"]])
            rate.sleep()
    
    def setPause(self, msg):
        self.pause = msg.data

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

    def getEyebrown(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyebrowRightHeight"]] = data[0]
        self.motors[MOTORS_IDX["EyebrowLeftHeight"]] = data[1]
        self.motors[MOTORS_IDX["EyebrowRightAngle"]] = data[2]
        self.motors[MOTORS_IDX["EyebrowLeftAngle"]] = data[3]

    def getNeck(self, msg):
       data = msg.data
       self.motors[MOTORS_IDX["NeckHorizontal"]] = (data[0] * 3.1415)/180
       self.motors[MOTORS_IDX["NeckVertical"]] = (data[1] * 3.1415)/180

if __name__ == '__main__':
    try:
        dataflowEnable()
    except rospy.ROSInterruptException:
        pass