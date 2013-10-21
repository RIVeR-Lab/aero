// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace aero_gazebo
{

class AeroHWSim : public gazebo_ros_control::RobotHWSim
{
private:
  double left_velocity_command_;
  double right_velocity_command_;

  double left_position_;
  double left_velocity_;
  double left_effort_;

  double right_position_;
  double right_velocity_;
  double right_effort_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  gazebo::physics::JointPtr front_left_joint_;
  gazebo::physics::JointPtr back_left_joint_;
  gazebo::physics::JointPtr front_right_joint_;
  gazebo::physics::JointPtr back_right_joint_;

public:
  bool initSim(
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    front_left_joint_ = parent_model->GetJoint("joint_front_left_wheel");
    back_left_joint_ = parent_model->GetJoint("joint_back_left_wheel");
    front_right_joint_ = parent_model->GetJoint("joint_front_right_wheel");
    back_right_joint_ = parent_model->GetJoint("joint_back_right_wheel");

    js_interface_.registerHandle(hardware_interface::JointStateHandle(
          "left_wheels_motor", &left_position_, &left_velocity_, &left_effort_));
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
          "right_wheels_motor", &right_position_, &right_velocity_, &right_effort_));
    vj_interface_.registerHandle(hardware_interface::JointHandle(
	 js_interface_.getHandle("left_wheels_motor"), &left_velocity_command_));
    vj_interface_.registerHandle(hardware_interface::JointHandle(
	 js_interface_.getHandle("right_wheels_motor"), &right_velocity_command_));

    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);

    return true;
  }

  void readSim(ros::Time time, ros::Duration period)
  {
    //only read data from the front wheel joints
    left_position_ += angles::shortest_angular_distance(left_position_,
	front_left_joint_->GetAngle(0).Radian());
    left_velocity_ = front_left_joint_->GetVelocity(0);
    left_effort_ = front_left_joint_->GetForce((unsigned int)(0));

    right_position_ += angles::shortest_angular_distance(right_position_,
	 front_right_joint_->GetAngle(0).Radian());
    right_velocity_ = front_right_joint_->GetVelocity(0);
    right_effort_ = front_right_joint_->GetForce((unsigned int)(0));
  }

  void writeSim(ros::Time time, ros::Duration period)
  {
    double torque_ = 100.0;
    front_left_joint_->SetVelocity(0, left_velocity_command_);
    front_left_joint_->SetMaxForce( 0, torque_ );
    back_left_joint_->SetVelocity(0, left_velocity_command_);
    back_left_joint_->SetMaxForce( 0, torque_ );
    front_right_joint_->SetVelocity(0, right_velocity_command_);
    front_right_joint_->SetMaxForce( 0, torque_ );
    back_right_joint_->SetVelocity(0, right_velocity_command_);
    back_right_joint_->SetMaxForce( 0, torque_ );

  }

};

typedef boost::shared_ptr<AeroHWSim> AeroHWSimPtr;

}

// \todo PLUGINLIB_DECLARE_CLASS has been deprecated. Replace it with PLUGINLIB_EXPORT_CLASS.
PLUGINLIB_EXPORT_CLASS(
  aero_gazebo::AeroHWSim,
  gazebo_ros_control::RobotHWSim)
