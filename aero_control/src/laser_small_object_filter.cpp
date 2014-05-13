#include <aero_control/laser_small_object_filter.h>
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace aero_control{
  using namespace boost::accumulators;

  LaserSmallObjectFilter::LaserSmallObjectFilter(){
  }
  LaserSmallObjectFilter::~LaserSmallObjectFilter(){
  }

  bool LaserSmallObjectFilter::configure(){
    if(!getParam("max_object_size", max_object_size_)){
      ROS_ERROR("Laser Small Object Filter needs max_object_size to be set");
      return false;
    }
    if(!getParam("min_surrounding", min_surrounding_)){
      ROS_ERROR("Laser Small Object Filter needs min_surrounding to be set");
      return false;
    }
    if(!getParam("max_depth_variance", max_depth_variance_)){
      ROS_ERROR("Laser Small Object Filter needs max_depth_variance to be set");
      return false;
    }
    return true;
  }

  bool LaserSmallObjectFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out){
    scan_out = scan_in;
    for(int i = 0; i<scan_out.ranges.size()-2*min_surrounding_-1; ++i){
      accumulator_set<double, stats<tag::mean, tag::variance(lazy)> > left_acc;
      for(int j = 0; j<min_surrounding_; ++j)
	left_acc(scan_out.ranges[i+j]);
      if(variance(left_acc) > max_depth_variance_)
      	continue;
      for(int size = max_object_size_; size>0; --size){//start with the largest size
        accumulator_set<double, stats<tag::mean, tag::variance(lazy)> > obj_acc;
        for(int j = 0; j<size; ++j)
	  obj_acc(scan_out.ranges[i+min_surrounding_+j]);
        if(variance(obj_acc) > max_depth_variance_)
	  continue;

        accumulator_set<double, stats<tag::mean, tag::variance(lazy)> > surrounding_acc;
        for(int j = 0; j<min_surrounding_; ++j)//add left
	  surrounding_acc(scan_out.ranges[i+j]);
        for(int j = 0; j<min_surrounding_; ++j)//add right
	  surrounding_acc(scan_out.ranges[i+min_surrounding_+size+j]);
        if(variance(surrounding_acc) > max_depth_variance_)
	  continue;

	//all variances are within tolerance
        for(int j = 0; j<size; ++j)
	  scan_out.ranges[i+min_surrounding_+j] = INFINITY;
	break;//don't bother checking smaller sizes
      }
    }
    return true;
  }

}

PLUGINLIB_DECLARE_CLASS(aero_control, LaserSmallObjectFilter, aero_control::LaserSmallObjectFilter, filters::FilterBase<sensor_msgs::LaserScan>)
