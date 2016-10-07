#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32


#init ros things
rospy.init_node('test')
fl_pub = rospy.Publisher("front_left", Float32)
fr_pub = rospy.Publisher("front_right", Float32)
bl_pub = rospy.Publisher("back_left", Float32)
br_pub = rospy.Publisher("back_right", Float32)

r = rospy.Rate(20)
c = 0
while not rospy.is_shutdown():
    option = int(c/100)
    # move forward
    if option == 0:
        fl_pub.publish(1.0)
        fr_pub.publish(1.0)
        bl_pub.publish(1.0)
        br_pub.publish(1.0)
    # move back
    if option == 1:
        fl_pub.publish(-1.0)
        fr_pub.publish(-1.0)
        bl_pub.publish(-1.0)
        br_pub.publish(-1.0)

    # move left
    if option == 2:
        fl_pub.publish(-1.0)
        fr_pub.publish(1.0)
        bl_pub.publish(1.0)
        br_pub.publish(-1.0)

    # move right
    if option == 3:
        fl_pub.publish(1.0)
        fr_pub.publish(-1.0)
        bl_pub.publish(-1.0)
        br_pub.publish(1.0)

    # spin
    if option == 4:
        fl_pub.publish(-1.0)
        fr_pub.publish(1.0)
        bl_pub.publish(-1.0)
        br_pub.publish(1.0)

    # spin
    if option == 5:
        fl_pub.publish(1.0)
        fr_pub.publish(-1.0)
        bl_pub.publish(1.0)
        br_pub.publish(-1.0)

    c += 1
    if c > 600:
        c = 0

    r.sleep()
