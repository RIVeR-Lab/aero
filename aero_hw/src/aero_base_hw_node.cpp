#include <ros/ros.h>
#include <ros/spinner.h>
#include <aero_hw/aero_base_hw.h>
#include <controller_manager/controller_manager.h>

int main( int argc, char** argv ){
  ros::init(argc, argv, "aero_base_hw");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double rate;
  pnh.param<double>("controller_rate", rate, 20);

  aero_hw::AeroBaseRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //won't run this is simulation so regular time is ok
  ros::Rate controller_rate(rate);
  ros::Time last = ros::Time::now();
  ROS_INFO_STREAM("Running controller at rate: " << rate);
  while (ros::ok())
  {
    robot.read();
    ros::Time now = ros::Time::now();
    cm.update(now, now-last);
    robot.write();
    last = now;
    ROS_INFO_STREAM_THROTTLE(5, "Controller Cycle Time: " << controller_rate.cycleTime().toSec());
    controller_rate.sleep();
  }

}
