#ifndef _PARABOT_HW_SIM_H__
#define _PARABOT_HW_SIM_H__

/* std include */
#include <boost/shared_ptr.hpp>
#include <thread>

/* gazebo */
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* ros */
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <control_toolbox/pid.h>

/* hardware interface */
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


class parabot_hw_sim: public hardware_interface::RobotHW
{
public:
  parabot_hw_sim(gazebo::physics::ModelPtr model, std::vector<std::string> joint_names,  std::string robot_namespace);
  parabot_hw_sim();
  ~parabot_hw_sim(){};
  bool initSim();

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

  void eStopActive(const bool active){};

  // virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void SetVelocity(const size_t index, const double &_vel);

private:
  // \brief Pointer to the model
  gazebo::physics::ModelPtr model_;
  std::vector<std::string> joint_names_;
  size_t dof_;
  std::string physics_type_;
   std::string robot_namespace_;

  // \brief Pointer to the joint
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  // \brief A PID controller for the joint.
  // common::PID pid_;

  /* ros interface */
  std::unique_ptr<ros::NodeHandle> rosNode;

  // ros subscriber
  ros::Subscriber rosSub;
  ros::Publisher  rosPub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;

  /* controller manager */
  boost::shared_ptr<controller_manager::ControllerManager>controller_manager_;

  /* hardware interface */
  hardware_interface::JointStateInterface jnt_st_int_;
  hardware_interface::PositionJointInterface pos_jnt_int_;
  hardware_interface::VelocityJointInterface vel_jnt_int_;
  hardware_interface::EffortJointInterface eft_jnt_int_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;

  struct JOINT_INFO
  {
      double pos;
      double vel;
      double eft;
  };

  struct JOINT
  {
      JOINT_INFO state;
      JOINT_INFO cmd;
  };

  std::vector<JOINT> joint_;

};

#endif // !_PARABOT_HW_SIM_H__  