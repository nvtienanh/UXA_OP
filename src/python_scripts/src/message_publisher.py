#!/usr/bin/env python

import rospy

from deno_my_ros.msg import Complex

from random import random


rospy.init_node('message_publisher')

pub = rospy.Publisher('complex', Complex, queue_size=2)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    msg = Complex()
    msg.real = random()
    msg.imaginary = random()

    pub.publish(msg)
    rate.sleep()

