#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import sys

def main():
    rospy.init_node('stm_msgs_publisher_mr2', anonymous=True)

    args = sys.argv

    if len(args) != 5:
        print("Wrong arguments!!")
        print("ex) 0.5 0.7 0.0 BLUE")
        return

    initial_pose_pub = rospy.Publisher('/nucleo/initial_pose', Point, queue_size=10)
    court_mode_pub = rospy.Publisher('/nucleo/court_mode', Int8, queue_size=10)

    p = Point()

    p.x = float(args[1])
    p.y = float(args[2])
    p.z = float(args[3])

    uargs = args[4].upper()
    court_mode = Int8()
    if uargs == "RED":
        court_mode.data = 0
    elif uargs == "BLUE":
        court_mode.data = 1
    elif uargs == "RED_MT":
        court_mode.data = 2
    elif uargs == "BLUE_MT":
        court_mode.data = 3
    else:
        print("Wrong court mode!")
        print("ex) RED or BLUE or RED_MT or BLUE_MT")
        return
    
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        initial_pose_pub.publish(p)
        court_mode_pub.publish(court_mode)
        print("Success publish!")
        r.sleep()

if __name__ == '__main__':
    main()