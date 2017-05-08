#!/usr/bin/env python
# BEGIN ALL

import rospy
from std_msgs.msg import Float64


rospy.init_node('cmd_joint_pub')

# BEGIN PUBLISER
pub = rospy.Publisher('/nimbro_op/right_shoulder_pitch_position_controller/command', Float64, queue_size=1)
# END PUBLISHER

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    joint = 0.6
    pub.publish(joint)
    rate.sleep()
#END ALL