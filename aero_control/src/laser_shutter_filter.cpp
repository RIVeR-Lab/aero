#include <aero_control/laser_shutter_filter.h>
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>

namespace aero_control{

  LaserShutterFilter::LaserShutterFilter():shutter_enabled_(false){
  }
  LaserShutterFilter::~LaserShutterFilter(){
  }

  bool LaserShutterFilter::configure(){
    ros::NodeHandle nh;
    enable_sub_ = nh.subscribe<std_msgs::Bool>("laser_shutter", 1, &LaserShutterFilter::shutter_callback, this);
    return true;
  }
  void LaserShutterFilter::shutter_callback(const std_msgs::Bool::ConstPtr& msg){
    shutter_enabled_ = msg->data;
  }


  bool LaserShutterFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out){
    if(shutter_enabled_){
      scan_out = scan_in;
      for(int i = 0; i<scan_out.ranges.size(); ++i){
        scan_out.ranges[i] = INFINITY;
      }
    }
    else{
      scan_out = scan_in;
    }
    return true;
  }

}

PLUGINLIB_DECLARE_CLASS(aero_control, LaserShutterFilter, aero_control::LaserShutterFilter, filters::FilterBase<sensor_msgs::LaserScan>)
