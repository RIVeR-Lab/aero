#ifndef LASER_SMALL_OBJECT_FILTER_H
#define LASER_SMALL_OBJECT_FILTER_H

#include <map>
#include <iostream>
#include <sstream>

#include <sensor_msgs/LaserScan.h>
#include <filters/filter_chain.h>

namespace aero_control{

  class LaserSmallObjectFilter : public filters::FilterBase<sensor_msgs::LaserScan> 
    {
    public:
      LaserSmallObjectFilter();
      ~LaserSmallObjectFilter();

      bool configure();

      bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

    private:
      int max_object_size_;
      int min_surrounding_;
      double max_depth_variance_;
    };

}

#endif
