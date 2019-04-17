#!/usr/bin/env python

#############################################################################
# Copyright 2015 Right Hand Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#############################################################################

__author__ = 'Duong Nguyen'
__email__ = 'dnguyen2@wpi.edu'

# from string import lstrip
# from math import pi

# from os.path import join
# import yaml

# from dynamixel_msgs.msg import JointState
# import rospkg
# import rospy
# from std_srvs.srv import Empty
# from rqt_service.srv import SendTwoInt

# from reflex_hand import ReflexHand
# from reflex_sf_motor import ReflexSFMotor
import reflex_msgs.msg
import rospy
from std_msgs.msg import Float64MultiArray,Float64,MultiArrayDimension,MultiArrayLayout

class HandVisualizer:
    def __init__(self):
        rospy.init_node('hand_visualizer')
        self.hand_state_sub = rospy.Subscriber('/reflex_sf/hand_state', reflex_msgs.msg.Hand, self.publish_finger_to_visualizer,queue_size=10)
        self.hand_state_pub = rospy.Publisher('/r2d2_head_controller/command', Float64MultiArray, queue_size=10)
        self.data = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0]
        self.mess_dim = MultiArrayDimension()
        self.mess_dim.label = 'why'
        self.mess_dim.size = 9
        self.mess_dim.stride = 1

        self.mess_layout = MultiArrayLayout()
        self.mess_layout.dim.append(self.mess_dim)
        self.mess_layout.data_offset = 0

    def publish_finger_to_visualizer(self,handData):
        #print(handData)
        self.data[0] = handData.motor[0].joint_angle
        self.data[1] = handData.motor[1].joint_angle
        self.data[2] = handData.motor[2].joint_angle
        self.data[3] = handData.motor[3].joint_angle
        self.data[4] = -handData.motor[3].joint_angle
        self.data[5] = 0
        self.data[6] = handData.motor[0].joint_angle/4
        self.data[7] = handData.motor[1].joint_angle/4
        self.data[8] = handData.motor[2].joint_angle/4
        
        mess = Float64MultiArray(self.mess_layout,self.data)
        
        self.hand_state_pub.publish(mess)
        


def main():
    rospy.sleep(4.0)  # To allow services and parameters to load
    handvisualizer = HandVisualizer()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        #hand._publish_hand_state()
        r.sleep()


if __name__ == '__main__':
    main()
