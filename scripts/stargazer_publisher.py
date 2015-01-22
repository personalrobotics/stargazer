#!/usr/bin/env python
from stargazer import StarGazer

import roslib
roslib.load_manifest('stargazer')
import rospy
import tf

FRAME_ROBOT = '/herb_base1'
FRAME_FIXED = '/map1'

class StarGazerNode:
    def __init__(self):
        self.stargazer = StarGazer(callback_global=self.callback_publish)
        
    def callback_publish(self, pose_dict):
        broadcaster = tf.TransformBroadcaster()
        for matrix in pose_dict.values():
            cartesian  = matrix[0:3,3]
            quaternion = tf.transformations.quaternion_from_matrix(matrix)
            broadcaster.sendTransform(cartesian,
                                      quaternion,
                                      rospy.Time.now(),
                                      FRAME_ROBOT,
                                      FRAME_FIXED)

if __name__ == '__main__':
    rospy.init_node('stargazer')
    node = StarGazerNode()
    rospy.spin()
