#include "parabot_hw_sim.h"

parabot_hw_sim::parabot_hw_sim(gazebo::physics::ModelPtr model, std::vector<std::string> joint_names, std::string robot_namespace)
  : model_(model), joint_names_(joint_names), robot_namespace_(robot_namespace)
{
  ROS_INFO("success_info");
  dof_ = joint_names_.size();
  std::cerr << model_->GetName() << "\n";
}
parabot_hw_sim::parabot_hw_sim()
{
  ROS_INFO("success_info");
}


bool parabot_hw_sim::initSim()
{
  joint_.resize(dof_);
  pid_controllers_.resize(dof_);
  for (size_t joint_id = 0; joint_id < dof_; joint_id++)
  {
      // hardware_interface init
      hardware_interface::JointStateHandle joint_state_handle(
          joint_names_[joint_id],&joint_[joint_id].state.pos,&joint_[joint_id].state.vel,&joint_[joint_id].state.eft
      );
      jnt_st_int_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle_position(
          jnt_st_int_.getHandle(joint_names_[joint_id]),&joint_[joint_id].cmd.pos
      );
      pos_jnt_int_.registerHandle(joint_handle_position);

      hardware_interface::JointHandle joint_handle_velocity(
          jnt_st_int_.getHandle(joint_names_[joint_id]),&joint_[joint_id].cmd.vel
      );
      vel_jnt_int_.registerHandle(joint_handle_velocity);

      hardware_interface::JointHandle joint_handle_effort(
          jnt_st_int_.getHandle(joint_names_[joint_id]),&joint_[joint_id].cmd.eft
      );
      eft_jnt_int_.registerHandle(joint_handle_effort);

      // gazebo api init
      gazebo::physics::JointPtr jointPtr = model_->GetJoint(joint_names_[joint_id]);
      if(!jointPtr)
      {
        ROS_ERROR_STREAM_NAMED("parabot_hw_sim", "This robot has a joint named \"" << joint_names_[joint_id]
          << "\" which is not in the gazebo model.");
        return false;
      }
      // gazebo joint interface
      sim_joints_.push_back(jointPtr);

      // Initialize the PID controller. 
      std::stringstream ss;
      ss << robot_namespace_ << "/joint" << joint_id+1 << "_position_controller/pid";
      const ros::NodeHandle nh(ss.str());
      if (pid_controllers_[joint_id].init(nh))
      {
        ROS_INFO("PID controller init success");
      }
      else
      {
        ROS_ERROR("PID controller not init success");
      }
  }


  registerInterface(&jnt_st_int_);
  registerInterface(&pos_jnt_int_);
  registerInterface(&vel_jnt_int_);
  registerInterface(&eft_jnt_int_);

  // Initialize emergency stop code
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
  physics_type_ = physics->GetType();

  return true;
}

void parabot_hw_sim::readSim(ros::Time time, ros::Duration period)
{
  for (size_t i = 0; i < this->sim_joints_.size(); i++)
  {
    joint_[i].state.pos = this->sim_joints_[i]->Position();
    joint_[i].state.vel = this->sim_joints_[i]->GetVelocity(0);
    joint_[i].state.eft = this->sim_joints_[i]->GetForce(0);
  }
  // std::cerr << "---------------------------------------------------------------"<<"\r\n";
  // std::cerr << "state of joint1 ["<< joint_[0].state.pos << " | " << joint_[0].state.vel   << " | " << joint_[0].state.eft << "]\r\n"
  //           << "state of joint2 ["<< joint_[1].state.pos << " | " << joint_[1].state.vel   << " | " << joint_[1].state.eft << "]\r\n"
  //           << "state of joint3 ["<< joint_[2].state.pos << " | " << joint_[2].state.vel   << " | " << joint_[2].state.eft << "]\r\n";
}

void parabot_hw_sim::writeSim(ros::Time time, ros::Duration period)
{
  for (size_t i = 0; i < this->sim_joints_.size(); i++)
  {
    // sim_joints_[i]->SetForce(0, 100);
    // sim_joints_[i]->SetForce(0, joint_[i].cmd.eft);
    double error;
    error = joint_[i].cmd.pos - joint_[i].state.pos;

    const double effort = pid_controllers_[i].computeCommand(error, period);
                                
    sim_joints_[i]->SetForce(0, effort);
  }
  
  // std::cerr << "cmd of joint1 ["<< joint_[0].cmd.pos << " | " << joint_[0].cmd.vel   << " | " << joint_[0].cmd.eft << "]\r\n"
  //           << "cmd of joint2 ["<< joint_[1].cmd.pos << " | " << joint_[1].cmd.vel   << " | " << joint_[1].cmd.eft << "]\r\n"
  //           << "cmd of joint3 ["<< joint_[2].cmd.pos << " | " << joint_[2].cmd.vel   << " | " << joint_[2].cmd.eft << "]\r\n";
}

void parabot_hw_sim::SetVelocity(const size_t _index, const double &_vel)
{
  // Set the joint's target velocity
  this->model_->GetJointController()->SetVelocityTarget(
    this->sim_joints_[_index]->GetScopedName(), _vel);
}