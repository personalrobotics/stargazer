#!/usr/bin/env python
import rospy
import numpy
import tf
from stargazer import StarGazer
from geometry_msgs.msg import (Point, Quaternion, Pose, PoseArray,
                               PoseWithCovariance, PoseWithCovarianceStamped)


def tf_to_matrix(trans, rot):
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    return numpy.dot(trans_mat, rot_mat)

def matrix_to_tf(mat):
    trans = tf.transformations.translation_from_matrix(mat)
    rot = tf.transformations.quaternion_from_matrix(mat)
    return trans, rot

def matrix_to_pose(mat):
    trans, rot = matrix_to_tf(mat)
    return Pose(
        position=Point(*trans),
        orientation=Quaternion(*rot)
    )


class StarGazerNode(object):
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('robot_pose', PoseWithCovarianceStamped)
        self.pose_array_pub = rospy.Publisher('robot_pose_array', PoseArray)
        self.unknown_ids = set()

    def run(self):
        args = {
            'device': rospy.get_param('~device'),
            'marker_map': rospy.get_param('~marker_map', {}),
            'callback_global': self.callback_global,
            'callback_local': self.callback_local,
        }
        parameters = self.get_options()

        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', 'map')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_link')
        self.stargazer_frame_id = rospy.get_param('~stargazer_frame_id', 'stargazer')
        self.marker_frame_prefix = rospy.get_param('~marker_frame_prefix', 'stargazer/')

        with StarGazer(**args) as stargazer:
            # The StarGazer might be streaming data. Turn off streaming mode.
            stargazer.stop_streaming()

            # Set all parameters, possibly to their default values. This is the
            # safest option because the parameters can be corrupted when the
            # StarGazer is powered off.
            for name, value in parameters.iteritems():
                stargazer.set_parameter(name, value)

            # Start streaming. ROS messages will be published in callbacks.
            stargazer.start_streaming()
            rospy.spin()

            # Stop streaming. Try to clean up after ourselves.
            stargazer.stop_streaming()

    def callback_global(self, pose_dict, unknown_ids):
        stamp = rospy.Time.now()
            
        # Print a warning about unmapped IDs.
        for unknown_id in unknown_ids - self.unknown_ids:
            rospy.logwarn('Detected marker ID %s that is not in the map.', unknown_id)
            self.unknown_ids.add(unknown_id)

        # Find the transform from the Stargazer to the robot.
        try:
            Tstargazer_robot = tf_to_matrix(
                *self.tf_listener.lookupTransform(
                    self.stargazer_frame_id, self.robot_frame_id, stamp)
            )
        except tf.Exception as e:
            rospy.logwarn('Failed looking up transform from "%s" to "%s": %s.',
                self.stargazer_frame_id, self.robot_frame_id, str(e))
            return

        # Publish the poses as ROS messages. Also publish an array of the robot
        # poses predicted from each marker. This is useful for visualization.
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = stamp
        pose_array_msg.header.frame_id = self.fixed_frame_id

        for marker_id, Tmap_stargazer in pose_dict.iteritems():
            # Convert the 'map -> stargazer' transform into a 'map -> robot' pose.
            Tmap_robot = numpy.dot(Tmap_stargazer, Tstargazer_robot)
            pose_msg = matrix_to_pose(Tmap_robot)
            pose_array_msg.poses.append(pose_msg)

            # Publish the output to a ROS message.
            pose_cov_msg = PoseWithCovarianceStamped()
            pose_cov_msg.header.stamp = stamp
            pose_cov_msg.header.frame_id = self.fixed_frame_id
            pose_cov_msg.pose.pose = pose_msg
            # TODO: Set the covariance.
            pose_cov_msg.pose.covariance = [ 0. ] * 36
            self.pose_pub.publish(pose_cov_msg)

        self.pose_array_pub.publish(pose_array_msg)
        

    def callback_local(self, pose_dict):
        stamp = rospy.Time.now()

        for marker_id, pose in pose_dict.iteritems():
            cartesian = pose[0:3, 3]
            quaternion = tf.transformations.quaternion_from_matrix(pose)

            frame_id = '{:s}/marker_{:d}'.format(self.marker_frame_prefix, marker_id)
            self.tf_broadcaster.sendTransform(
                cartesian, quaternion, stamp, frame_id, self.stargazer_frame_id
            )

    def get_options(self):
        """ Gets StarGazer options from the ROS parameter server.
        """
        options = {}

        # Threshold level to reject external turbulence shown in image; depend
        # on surroundings. Recommended value is ranging from 210 to 240.
        options['ThrVal'] = rospy.get_param('~ThrVal', 210)

        # Distance from a StarGazer to a landmark; used when wanting to input
        # manually the height (in millimeters).
        options['MarkHeight'] = rospy.get_param('~MarkHeight', 2400)

        # A total number of landmarks to be assigned under Map Mode.
        options['IDNum'] = rospy.get_param('~IDNum', 4)
        assert 0 <= options['IDNum'] < 4095

        # The number of reference ID under map mode.
        options['RefID'] = rospy.get_param('~RefID', 2)

        # To determine how to get ThrVal; There are Auto and Manual. 'Manual'
        # should be assigned to use input data and 'Auto' be assigned to use a
        # value calculated automatically in StarGazer
        options['ThrAlg'] = rospy.get_param('~ThrAlg', 'Manual')
        assert options['ThrAlg'] in ['Auto', 'Manual']

        # To set up landmark type by use. There are Home and Office. Home means
        # HL1-1 landmark (up to 31 IDs) and Office means HL2-1 (up to 4095
        # IDs).
        options['MarkType'] = rospy.get_param('~MarkType', 'Home')
        assert options['MarkType'] in ['Home', 'Office']

        # To setup landmark type by height. There are different landmark types
        # for height - HLDn-2 for MarkDim 1, HLDn-3 for MarkDim 2, and HLDn-4
        # for MarkDim 3.
        options['MarkDim'] = rospy.get_param('~MarkDim', 1)
        assert options['MarkDim'] in [1, 2, 3]

        # To determine whether map building is executed or not. There are Start
        # and Stop. If action under Map Mode is required, you should set the
        # parameter to 'Start' and start Map Building.
        options['MapMode'] = rospy.get_param('~MapMode', 'Stop')
        assert options['MapMode'] in ['Start', 'Stop']

        # To determine whether landmarks are used independently under Alone
        # Mode or not (dependently under Map Mode). There are Alone and Map.
        options['MarkMode'] = rospy.get_param('~MarkMode', 'Alone')
        assert options['MarkMode'] in ['Alone', 'Map']

        return options

if __name__ == '__main__':
    rospy.init_node('stargazer')
    node = StarGazerNode()
    node.run()

