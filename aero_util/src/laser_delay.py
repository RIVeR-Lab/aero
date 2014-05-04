#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import *

def callback(data):
    rospy.loginfo("dt=%f", (rospy.get_rostime()-data.header.stamp).to_sec())

def main():
    rospy.init_node('laser_delay', anonymous=True)
    rospy.Subscriber("/aero/laser", LaserScan, callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
