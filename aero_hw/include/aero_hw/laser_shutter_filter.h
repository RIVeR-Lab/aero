#ifndef LASER_SHUTTER_FILTER_H
#define LASER_SHUTTER_FILTER_H

#include <map>
#include <iostream>
#include <sstream>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <filters/filter_chain.h>

namespace aero_hw{

  class LaserShutterFilter : public filters::FilterBase<sensor_msgs::LaserScan> 
    {
    public:
      LaserShutterFilter();
      ~LaserShutterFilter();

      bool configure();

      bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

    private:
      ros::Subscriber enable_sub_;
      void shutter_callback(const std_msgs::Bool::ConstPtr& msg);
      bool shutter_enabled_;  
    };

}

#endif
