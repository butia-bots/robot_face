#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Float64MultiArray, Int16
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, PointStamped
from butia_vision_msgs.srv import LookAtDescription3D, LookAtDescription3DRequest, LookAtDescription3DResponse
from butia_vision_msgs.msg import  Recognitions3D, Description3D

import tf2_ros
import tf2_geometry_msgs

#TODO: this should be in a generic file
EMOTIONS = {
    "standard": 0,
    "happy": 1,
    "sad": 2,
    "rage": 3,
    "scared": 4,
}

class neckController():
    STATES = {
        'EMOTION':      0,
        'HAND_UPDATED': 1,
        'LOOKAT':       2
    }

    def __init__(self):
        rospy.init_node('neckController', anonymous=False)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.neck_pub = rospy.Publisher("neck", Float64MultiArray, queue_size = 1)

        self.sub_update_neck = rospy.Subscriber("updateNeck", Float64MultiArray, self.getNeck_st,
                                                queue_size=1)
        self.sub_update_neck_by_point = rospy.Subscriber("updateNeckByPoint", PointStamped,
                                                         self.getNeckByPoint_st, queue_size=1)
        
        self.sub_emotion = rospy.Subscriber('emotion', Int16, self.getEmotion_st)

        self.start_lookat_service = rospy.Service('lookat_start', LookAtDescription3D, self.lookAtStart)
        self.stop_lookat_service = rospy.Service('lookat_stop', Empty, self.lookAtStop)
        self.lookat_sub = None
        self.lookat_description_identifier = None
        self.lookat_neck = None

        self.publish = True
        
        self.emotion = 0
        self.neck_updated = None

        self.horizontal = None
        self.vertical = None

        self.state = neckController.STATES['EMOTION']

        self._readParameters()

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.getOutput()

            self.publishNeck()

            rate.sleep()

    def computeTFTransform(self, header):
        try:
            transform = self.tf_buffer.lookup_transform('camera_link_static', header.frame_id, header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
    
        return transform
    
    def computeNeckStateByPoint(self, point):
        horizontal = math.pi + math.atan2(point.y, point.x)
        dist = math.sqrt(point.x**2 + point.y**2)
        vertical = math.pi + math.atan2(point.z, dist)

        return [math.degrees(horizontal), math.degrees(vertical)]
        
    def getOutput(self):
        if self.state == neckController.STATES['EMOTION']:
            if(self.emotion == EMOTIONS["standard"]):                     
                self.horizontal = self.horizontal_standard_params                  
                self.vertical = self.vertical_standard_params                
            elif(self.emotion == EMOTIONS["happy"]):                        
                self.horizontal = self.horizontal_happy_params                     
                self.vertical = self.vertical_happy_params                        
            elif(self.emotion == EMOTIONS["sad"]):                         
                self.horizontal = self.horizontal_sad_params                   
                self.vertical = self.vertical_sad_params                          
            elif(self.emotion == EMOTIONS["rage"]):                       
                self.horizontal = self.horizontal_rage_params                    
                self.vertical = self.vertical_rage_params                     
            elif(self.emotion == EMOTIONS["scared"]):                      
                self.horizontal = self.horizontal_scared_params                  
                self.vertical = self.vertical_scared_params
        elif self.state == neckController.STATES['HAND_UPDATED']:
            if self.neck_updated is not None:
                self.horizontal, self.vertical = self.neck_updated
        elif self.state == neckController.STATES['LOOKAT']:
            if self.lookat_neck is not None:
                self.horizontal, self.vertical = self.lookat_neck

    def getEmotion_st(self, msg):
        self.lookAtStop(None)
        self.emotion = msg.data
        self.state = neckController.STATES['EMOTION']
        self.publish = True

    def getNeckByPoint_st(self, msg):
        self.lookAtStop(None)
        transform = self.computeTFTransform(msg.header)
        ps = tf2_geometry_msgs.do_transform_point(msg, transform).point
        self.neck_updated = self.computeNeckStateByPoint(ps)
        self.state = neckController.STATES['HAND_UPDATED']
        self.publish = True

    def getNeck_st(self, msg):
        self.lookAtStop(None)
        self.neck_updated = [float(msg.data[0]), float(msg.data[1])]
        self.state = neckController.STATES['HAND_UPDATED']
        self.publish = True

    def publishNeck(self):
        if self.publish:
            neck_data = Float64MultiArray()
            neck_data.data = [float(self.horizontal), float(self.vertical)]

            self.neck_pub.publish(neck_data)
            self.publish = False
    
    def getCloserDescription(self, descriptions):
        min_desc = None
        min_dist = float('inf')
        for desc in descriptions:
            p = desc.bbox.center.position
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if dist < min_dist:
                min_desc = desc
                min_dist = dist
        
        return min_desc

    # TODO: implement for another parameters like global_id or local_id
    def selectDescription(self, descriptions):
        selected_descriptions = []

        desired_label = self.lookat_description_identifier['label']

        if desired_label != '':
            for desc in descriptions:
                if desc.label == desired_label:
                   selected_descriptions.append(desc)
        else:
            selected_descriptions = descriptions

        desc = self.getCloserDescription(selected_descriptions)

        return desc
    
    def lookAt_st(self, msg):
        selected_desc = self.selectDescription(msg.descriptions)

        if selected_desc is not None:

            header = selected_desc.poses_header

            transform = self.computeTFTransform(header)

            lookat_pose = PoseStamped()
            lookat_pose.header = header
            lookat_pose.pose = selected_desc.bbox.center

            ps = tf2_geometry_msgs.do_transform_pose(lookat_pose, transform).pose.position
            
            self.lookat_neck = self.computeNeckStateByPoint(ps)
            
            self.publish = True

    def lookAtStart(self, req):
        self.lookat_description_identifier = {'global_id': req.global_id, 'id': req.id, 'label': req.label}
        self.lookat_sub = rospy.Subscriber(req.recognitions3d_topic, Recognitions3D, self.lookAt_st, queue_size=1)
        self.state = neckController.STATES['LOOKAT']
        return LookAtDescription3DResponse()

    def lookAtStop(self, req):
        self.state = neckController.STATES['EMOTION']
        if self.lookat_sub is not None:
            self.lookat_sub.unregister()
            self.lookat_sub = None
        self.lookat_description_identifier = None
        self.publish = True
        return EmptyResponse()

    def _readParameters(self):
        self.horizontal_standard_params = rospy.get_param("butia_emotions/neck/standard/horizontal")
        self.vertical_standard_params = rospy.get_param("butia_emotions/neck/standard/vertical")

        self.horizontal_happy_params = rospy.get_param("butia_emotions/neck/happy/horizontal")
        self.vertical_happy_params = rospy.get_param("butia_emotions/neck/happy/vertical")

        self.horizontal_sad_params = rospy.get_param("butia_emotions/neck/sad/horizontal")
        self.vertical_sad_params = rospy.get_param("butia_emotions/neck/sad/vertical")

        self.horizontal_rage_params = rospy.get_param("butia_emotions/neck/rage/horizontal")
        self.vertical_rage_params = rospy.get_param("butia_emotions/neck/rage/vertical")

        self.horizontal_scared_params = rospy.get_param("butia_emotions/neck/scared/horizontal")
        self.vertical_scared_params = rospy.get_param("butia_emotions/neck/scared/vertical")

if __name__ == '__main__':
    try:
        neckController()
    except rospy.ROSInterruptException:
        pass