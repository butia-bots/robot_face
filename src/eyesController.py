#!/usr/bin/env python3
import rospy
import time
import threading
import os
from std_msgs.msg import Int16MultiArray, Float64MultiArray, Int16
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

class eyesEnable():
    def __init__(self):
        pub = rospy.Publisher('eye', Int16MultiArray, queue_size=10)
        rospy.init_node('eyesEnable', anonymous=False)
      #  self.sub_eye = rospy.Subscriber('updateEyes', Float64MultiArray, self.getEyes)
        self.sub_eye_emotion = rospy.Subscriber('emotion', Int16, self.getEyes_emotion)
        rate = rospy.Rate(50) 

        self.output = Int16MultiArray()
        self.output.data = []

        self._readParameters()

        self.emotion = 0
        self.xPosition = self.horizontal_standard_params
        self.yPosition = self.vertical_standard_params

        while not rospy.is_shutdown():
            self.getOutput()
            self.output.data = []
            self.output.data = [self.xPosition , self.yPosition]
            pub.publish(self.output)
            rate.sleep()
#
#   def getEyes(self, msg):
        #*** Qual a estrutura e l�gica da mensagem enviado pelo t�pico updateEyes?
 #       self.data = msg.data
  #      self.x = self.data[0]
   #     self.y = self.data[1]
    #    self.width = self.data[3]
     #   self.height = self.data[2]

      #  self.xPosition = abs(100 - mp.map(0, self.width, 0, 100, self.x))
      #  self.yPosition = mp.map(0, self.height, 0, 100, self.y)

    def getOutput(self):
        
        if(self.emotion == EMOTIONS["standard"]):                       # Standard
            self.xPosition = self.horizontal_standard_params                   # 
            self.yPosition = self.vertical_standard_params                     # 
        
        elif(self.emotion == EMOTIONS["happy"]):                        # Happy
            self.xPosition = self.horizontal_happy_params                      # 
            self.yPosition = self.vertical_happy_params                        # 
        
        elif(self.emotion == EMOTIONS["sad"]):                          # Sad
            self.xPosition = self.horizontal_sad_params                        # 
            self.yPosition = self.vertical_sad_params                          # 
       
        elif(self.emotion == EMOTIONS["rage"]):                         # Rage
            self.xPosition = self.horizontal_rage_params                       # 
            self.yPosition = self.vertical_rage_params                         # 
       
        elif(self.emotion == EMOTIONS["scared"]):                       # Scared
            self.xPosition = self.horizontal_scared_params          
            self.yPosition = self.vertical_scared_params                       # 

        elif(self.emotion == EMOTIONS["doubt"]):                       # doubt
            self.xPosition = self.horizontal_doubt_params                     # 
            self.yPosition = self.vertical_doubt_params                       # 

        elif(self.emotion == EMOTIONS["sleepy"]):                       # sleepy
            self.xPosition = self.horizontal_sleepy_params                     # 
            self.yPosition = self.vertical_sleepy_params                       # 
            

    def getEyes_emotion(self, msg):
        self.data=msg.data
        self.emotion=self.data


    def _readParameters(self):
        self.horizontal_standard_params = rospy.get_param("butia_emotions/eyes/standard/horizontal")
        self.vertical_standard_params = rospy.get_param("butia_emotions/eyes/standard/vertical")

        self.horizontal_happy_params = rospy.get_param("butia_emotions/eyes/happy/horizontal")
        self.vertical_happy_params = rospy.get_param("butia_emotions/eyes/happy/vertical")

        self.horizontal_sad_params = rospy.get_param("butia_emotions/eyes/sad/horizontal")
        self.vertical_sad_params = rospy.get_param("butia_emotions/eyes/sad/vertical")

        self.horizontal_rage_params = rospy.get_param("butia_emotions/eyes/rage/horizontal")
        self.vertical_rage_params = rospy.get_param("butia_emotions/eyes/rage/vertical")

        self.horizontal_scared_params = rospy.get_param("butia_emotions/eyes/scared/horizontal")
        self.vertical_scared_params = rospy.get_param("butia_emotions/eyes/scared/vertical")

        self.horizontal_doubt_params = rospy.get_param("butia_emotions/eyes/doubt/horizontal")
        self.vertical_doubt_params = rospy.get_param("butia_emotions/eyes/doubt/vertical")

        self.horizontal_sleepy_params = rospy.get_param("butia_emotions/eyes/sleepy/horizontal")
        self.vertical_sleepy_params = rospy.get_param("butia_emotions/eyes/sleepy/vertical")


if __name__ == '__main__':
    try:
        eyesEnable()
    except rospy.ROSInterruptException:
        pass