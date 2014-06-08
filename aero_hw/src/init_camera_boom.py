#!/usr/bin/env python 
import roslib; roslib.load_manifest('aero_hw')
import rospy
import actionlib
from device_driver_base.msg import *

def main():
    rospy.init_node('init_camera_boom')
    client = actionlib.SimpleActionClient('/aero/hd_camera_boom_control', SetMotorPositionAction)

    target_position = rospy.get_param("~target_position")
    rospy.loginfo('Sending boom to %f', target_position)

    client.wait_for_server()

    rospy.loginfo('Sending boom to %f', target_position)
    goal = SetMotorPositionGoal(position=target_position, max_velocity=200000)

    client.send_goal(goal)

    client.wait_for_result()

    rospy.loginfo('Sent boom to %s', str(client.get_result()))
    

if __name__ == '__main__':
    main()
