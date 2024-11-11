#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, Bool, Float64MultiArray
from PyDynamixel import DxlCommProtocol1, DxlCommProtocol2, JointProtocol1, JointProtocol2

MOTORS_IDX = {
    "EyebrowRightHeight": 0,
    "EyebrowLeftHeight": 1,
    "EyebrowRightAngle": 2,
    "EyebrowLeftAngle": 3,
    "EyelidRight": 4,
    "EyelidLeft": 5,
    "EyeHorizontal": 6,
    "EyeVertical": 7,
    "JawLeft": 8,
    "JawRight":9,
    "NeckHorizontal": 10,
    "NeckVertical": 11,
    "Pan": 12,
    "Tilt": 13
}

class dataflowEnable():
    def __init__(self, pause=False):
        self.pause = pause

        rospy.init_node('dataController', anonymous=False)
        rate = rospy.Rate(100) # 100hz

        self.min_horizontal = 120
        self.max_horizontal = 240
        self.min_vertical = 150
        self.max_vertical = 190

        try:
            self.neck_port = DxlCommProtocol2("/dev/ttyNECK")

            self.neckHorizontal = JointProtocol2(62)
            self.neckVertical = JointProtocol2(61)
            self.panJoint = JointProtocol2(8)
            self.tiltJoint = JointProtocol2(9)

            self.neck_port.attachJoint(self.neckVertical)
            self.neck_port.attachJoint(self.neckHorizontal)
            self.neck_port.attachJoint(self.panJoint)
            self.neck_port.attachJoint(self.tiltJoint)

            self.neckHorizontal.enableTorque()
            self.neckVertical.enableTorque()
            self.panJoint.enableTorque()
            self.tiltJoint.enableTorque()

            # Quanto menor o limit, maior a velocidade
            var_limit = 800
            self.neckHorizontal.setVelocityLimit(limit=var_limit)
            self.neckVertical.setVelocityLimit(limit=var_limit)
            self.panJoint.setVelocityLimit(limit=var_limit)
            self.tiltJoint.setVelocityLimit(limit=var_limit)
        except Exception as e:
            print("Neck port don't connected.")

        # Define the output vector
        self.motors = [0] * len(MOTORS_IDX.keys())

        self.motors[MOTORS_IDX["EyebrowRightHeight"]] = rospy.get_param("butia_emotions/eyebrown/standard/rightY")
        self.motors[MOTORS_IDX["EyebrowLeftHeight"]] = rospy.get_param("butia_emotions/eyebrown/standard/leftY")
        self.motors[MOTORS_IDX["EyebrowRightAngle"]] = rospy.get_param("butia_emotions/eyebrown/standard/rightRotation")
        self.motors[MOTORS_IDX["EyebrowLeftAngle"]] = rospy.get_param("butia_emotions/eyebrown/standard/leftRotation")
        self.motors[MOTORS_IDX["EyelidRight"]] = rospy.get_param("butia_emotions/eyelid/standard/right")
        self.motors[MOTORS_IDX["EyelidLeft"]] = rospy.get_param("butia_emotions/eyelid/standard/left")
        self.motors[MOTORS_IDX["EyeHorizontal"]] = rospy.get_param("butia_emotions/eyes/standard/horizontal")
        self.motors[MOTORS_IDX["EyeVertical"]] = rospy.get_param("butia_emotions/eyes/standard/vertical")
        self.motors[MOTORS_IDX["JawLeft"]] = rospy.get_param("butia_emotions/jaw/standard/left")
        self.motors[MOTORS_IDX["JawRight"]] = rospy.get_param("butia_emotions/jaw/standard/right")
        self.motors[MOTORS_IDX["NeckHorizontal"]] = np.pi
        self.motors[MOTORS_IDX["NeckVertical"]] = np.pi
        self.motors[MOTORS_IDX["Pan"]] = np.pi
        self.motors[MOTORS_IDX["Tilt"]] = np.pi

        self.seq = 0
        self.port = DxlCommProtocol1(commPort="/dev/ttyFACE")
        self.joint = JointProtocol1(128)
        self.port.attachJoint(self.joint)

        rospy.Subscriber("/emergency_button", Bool, self.setPause)

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

        self.joints_dict = {
            'horizontal_neck_joint': (0., 0., 0.),
            'vertical_neck_joint':   (0., 0., 0.),
            'head_pan_joint':        (0., 0., 0.),
            'head_tilt_joint':       (0., 0., 0.)
        }

        self.pub_neck = rospy.Publisher('/doris_head/joint_states', JointState, queue_size=10)

        while not rospy.is_shutdown():
            if not self.pause: 
                self.joint.writeValue(4, int(self.motors[MOTORS_IDX["EyelidRight"]]))
                self.joint.writeValue(5, int(self.motors[MOTORS_IDX["EyelidLeft"]]))
                self.joint.writeValue(8, int(self.motors[MOTORS_IDX["JawLeft"]])) #mouth
                self.joint.writeValue(9, int(self.motors[MOTORS_IDX["JawRight"]])) #mouth
                self.joint.writeValue(0, int(self.motors[MOTORS_IDX["EyebrowRightHeight"]]))
                self.joint.writeValue(1, int(self.motors[MOTORS_IDX["EyebrowLeftHeight"]]))
                self.joint.writeValue(2, int(self.motors[MOTORS_IDX["EyebrowRightAngle"]]))
                self.joint.writeValue(3, int(self.motors[MOTORS_IDX["EyebrowLeftAngle"]]))
                self.joint.writeValue(6, int(self.motors[MOTORS_IDX["EyeHorizontal"]]))
                self.joint.writeValue(7, int(self.motors[MOTORS_IDX["EyeVertical"]]))
                self.neckHorizontal.sendGoalAngle(self.motors[MOTORS_IDX["NeckHorizontal"]])
                self.neckVertical.sendGoalAngle(self.motors[MOTORS_IDX["NeckVertical"]])
                self.panJoint.sendGoalAngle(self.motors[MOTORS_IDX["Pan"]])
                self.tiltJoint.sendGoalAngle(self.motors[MOTORS_IDX["Tilt"]])
                self.updateJointsDict()
                self.publishJoints()
            rate.sleep()
    
    def updateJointsDict(self):
        pos_horizontal = self.neckHorizontal.receiveCurrAngle()
        pos_vertical = self.neckVertical.receiveCurrAngle()
        pos_pan = self.panJoint.receiveCurrAngle()
        pos_tilt = self.tiltJoint.receiveCurrAngle()

        self.joints_dict['horizontal_neck_joint'] = (pos_horizontal - np.pi, 0., 0.)
        self.joints_dict['head_pan_joint'] = (pos_pan - np.pi, 0., 0.)

        self.joints_dict['vertical_neck_joint'] = (-pos_vertical + np.pi, 0., 0.)
        self.joints_dict['head_tilt_joint'] = (pos_tilt - np.pi, 0., 0.)

    def publishJoints(self):
        msg = JointState()

        # Nome da joint no URDF do Kinect
        msg.header.seq = self.seq
        msg.header.stamp = rospy.get_rostime()
        
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for key, value in self.joints_dict.items():
            p, v, e = value
            msg.name.append(key)
            msg.position.append(p)
            msg.velocity.append(v)
            msg.effort.append(e)

        # Publica a mensagem
        self.pub_neck.publish(msg)

        # Incrementa seq
        self.seq += 1
    
    def setPause(self, msg):
        self.pause = not msg.data

    def getMouth(self, msg):
        data = msg.data
        #self.motors[0] = int(0.3059*self.data[0])
        #self.motors[10] = abs(100-data[0])
        self.motors[MOTORS_IDX["JawLeft"]] = data[0]
        self.motors[MOTORS_IDX["JawRight"]] = data[1]
        #self.motors[1] = data[1]

    def getEye(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyeHorizontal"]] = data[0]
        self.motors[MOTORS_IDX["EyeVertical"]] = data[1]
    
    def getEyelid(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyelidRight"]] = data[0]
        self.motors[MOTORS_IDX["EyelidLeft"]] = data[1]

    def getEyebrown(self, msg):
        data = msg.data
        self.motors[MOTORS_IDX["EyebrowRightHeight"]] = data[0]
        self.motors[MOTORS_IDX["EyebrowLeftHeight"]] = data[1]
        self.motors[MOTORS_IDX["EyebrowRightAngle"]] = data[2]
        self.motors[MOTORS_IDX["EyebrowLeftAngle"]] = data[3]

    def getNeck(self, msg):
       data = msg.data
       pos_horizontal = np.radians(min(self.max_horizontal, max(self.min_horizontal, data[0])))
       pos_vertical = np.radians(min(self.max_vertical, max(self.min_vertical, data[1])))

       self.motors[MOTORS_IDX["NeckHorizontal"]] = pos_horizontal
       self.motors[MOTORS_IDX["NeckVertical"]] = pos_vertical

       self.motors[MOTORS_IDX["Pan"]] = pos_horizontal
       self.motors[MOTORS_IDX["Tilt"]] = 2*np.pi - pos_vertical

if __name__ == '__main__':
    try:
        dataflowEnable()
    except rospy.ROSInterruptException:
        pass
