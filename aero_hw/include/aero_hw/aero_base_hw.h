#ifndef AERO_BASE_HW_H_
#define AERO_BASE_HW_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <safety_interface/safety_interface.h>
#include <boost/shared_ptr.hpp>
#include <river_ros_util/ros_util.h>
#include <river_ros_util/ros_control_util.h>
#include <roboteq_driver/roboteq_controller_hw.h>
#include <boost/assign/list_of.hpp>
#include <vector>

namespace aero_hw{

using namespace roboteq_driver;
using namespace boost::assign;

class AeroBaseRobot : public hardware_interface::RobotHW
{
public:
 AeroBaseRobot(ros::NodeHandle n = ros::NodeHandle(), std::string robot_ns="aero/"){
    memset(drive_joint_data, 0, sizeof(drive_joint_data));
    std::string boom_name = robot_ns+"boom_joint";
    vector<std::string> drive_names = list_of(robot_ns+"joint_front_left_wheel")(robot_ns+"joint_front_right_wheel");
    drive_trans = list_of<transmission_interface::SimpleTransmission>(-15.0)(-15.0);

    drive_motor_controller = boost::shared_ptr<RoboteqControllerHW>(new RoboteqControllerHW(n, "/dev/MTR", drive_names[0], 1800, 5000, drive_names[1], 1800, 5000, act_state_interface, act_vel_interface, safety_interface));
    

    for(int i = 0; i<drive_names.size(); ++i){
      ROS_INFO_STREAM(drive_names[i]);
      hardware_interface::ActuatorHandle actuator_handle = act_vel_interface.getHandle(drive_names[i]);
      act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle(drive_names[i],
                                                                 &drive_trans[i],
  							         river_ros_util::create_transmission_actuator_state_data(actuator_handle),
  							         drive_joint_data[i].transmission_state_data()));

      transmission_interface::ActuatorData a_cmd_data;
      transmission_interface::JointData j_cmd_data;
      hardware_interface::ActuatorHandle actuator_cmd_handle = act_vel_interface.getHandle(drive_names[i]);
      j_cmd_data.velocity.push_back(&drive_joint_data[i].cmd);
      a_cmd_data.velocity.push_back((double*)actuator_cmd_handle.getCommandPtr());
      jnt_to_act_vel.registerHandle(transmission_interface::JointToActuatorVelocityHandle(drive_names[i],
  								 &drive_trans[i],
  								 a_cmd_data,
  								 j_cmd_data));


      jnt_state_interface.registerHandle(drive_joint_data[i].state_handle(drive_names[i]));
      jnt_vel_interface.registerHandle(hardware_interface::JointHandle(jnt_state_interface.getHandle(drive_names[i]), &drive_joint_data[i].cmd));
    }

    jnt_state_interface.registerHandle(drive_joint_data[0].state_handle(robot_ns+"joint_back_left_wheel"));
    jnt_state_interface.registerHandle(drive_joint_data[1].state_handle(robot_ns+"joint_back_right_wheel"));

    jnt_state_interface.registerHandle(boom_joint_data.state_handle(boom_name));
    jnt_pos_interface.registerHandle(hardware_interface::JointHandle(jnt_state_interface.getHandle(boom_name), &boom_joint_data.cmd));

    registerInterface(&act_state_interface);
    registerInterface(&act_vel_interface);
    registerInterface(&act_pos_interface);
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&act_to_jnt_state);
    registerInterface(&jnt_to_act_vel);
    registerInterface(&safety_interface);
  }

  void read(){
    drive_motor_controller->read();
    act_to_jnt_state.propagate();
  }
  void write(){
    drive_motor_controller->write();
    jnt_to_act_vel.propagate();
  }

 private:
  boost::shared_ptr<RoboteqControllerHW> drive_motor_controller;  
  hardware_interface::ActuatorStateInterface act_state_interface;
  hardware_interface::VelocityActuatorInterface act_vel_interface;
  hardware_interface::PositionActuatorInterface act_pos_interface;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state;
  transmission_interface::JointToActuatorVelocityInterface jnt_to_act_vel;
  safety_interface::SafetyInterface safety_interface;

  std::vector<transmission_interface::SimpleTransmission> drive_trans;

  river_ros_util::JointData drive_joint_data[2];
  river_ros_util::JointData boom_joint_data;
};

}

#endif
