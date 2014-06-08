#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <safety_interface/SoftwareStop.h>


std::string stop_topic("stop");
std::string arduino_pause_topic("arduino_pause");


static ros::Publisher pause_pub;
static ros::Subscriber arduino_pause_sub;


bool last_pause_state = true;
bool is_first_time = true;

double time_of_toggle = 0;


void pauseCallback(const std_msgs::BoolConstPtr& is_paused){
  ROS_DEBUG_STREAM("Reveived software stop message from arduino: "<<is_paused->data);
  
  if(is_paused->data!=last_pause_state || is_first_time){
    safety_interface::SoftwareStop msg;
    msg.stop = is_paused->data;
    msg.header.stamp = ros::Time::now();
	    
    if(msg.stop){
      msg.message = "Hardware pause was pressed";
      time_of_toggle = msg.header.stamp.toSec();
    }
    else{
      msg.message = "Hardware pause was released";
      if(ros::Time::now().toSec() - time_of_toggle < 5 && !is_first_time){
	msg.message = "Hardware pause is released, but delay is in effect";
	msg.stop = true;
      }
      else{
	msg.message = "Hardware pause was pressed";
      }
    }

    last_pause_state = msg.stop;
    is_first_time = false;
    pause_pub.publish(msg);
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_arduino_controller");
	ros::NodeHandle nh;

	arduino_pause_sub = nh.subscribe(arduino_pause_topic, 1000, pauseCallback);
	pause_pub = nh.advertise<safety_interface::SoftwareStop>(stop_topic, 1000, true);

	ros::spin();
}
