#!/usr/bin/env python
import deferred
import rospy
import tf
from stargazer import StarGazer

FRAME_ROBOT = '/herb_base1'
FRAME_FIXED = '/map1'

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

def get_options():
    """ Gets StarGazer options from the ROS parameter server.
    """

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

    device = rospy.get_param('~device')
    marker_map = rospy.get_param('~marker_map')
    parameters = get_options()

    args = {
        'device': device,
        'marker_map': marker_map,
        'callback_global': callback_publish,
    }

    with StarGazer(**args) as stargazer:
        # The StarGazer might be streaming data. Turn off streaming mode.
        stargazer.send_command('CalcStop')

        # Set all parameters, possibly to their default values. This is the
        # safest option because the parameters can be corrupted when the
        # StarGazer is powered off.
        for name, value in parameter.iteritems():
            stargazer.set_parameter(key, value)

        # Start streaming data from the StarGazer. data 
        stargazer.start_streaming()
        rospy.spin()
        stargazer.stop_streaming()

