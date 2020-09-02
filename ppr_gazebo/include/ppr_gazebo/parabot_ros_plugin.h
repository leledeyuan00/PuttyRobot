#ifndef _PARABOT_ROS_PLUGIN__
#define _PRARBOT_ROS_PLUGIN__

/* std include */
#include <boost/shared_ptr.hpp>
#include <thread>

/* ros */
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Bool.h>

/* Gazebo */
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

// ros_control
#include <gazebo_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>

#include "parabot_hw_sim.h"


namespace gazebo
{
  class ParabotPlugin : public ModelPlugin
  {
  public:
    ParabotPlugin(){};
    ~ParabotPlugin(){};

    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr sdf);

    void Update();

    virtual void Reset();

  private:

    /* ros declare */
    ros::NodeHandle rosNode;

    /* gazebo plugin */
    gazebo::physics::ModelPtr model_;
    sdf::ElementPtr sdf_;

    std::string joint_parse_;
    std::vector<std::string> joint_names_;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;
    

    boost::shared_ptr<parabot_hw_sim> parabot_hw_sim_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    
    // Timing
    ros::Duration control_period_;
    ros::Time last_update_sim_time_ros_;
    ros::Time last_write_sim_time_ros_;

    // e stop active is true if the emergency stop is active
    bool e_stop_active_, last_e_stop_active_;
    ros::Subscriber e_stop_sub_; // Emergency sub

    /* function */
    void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ParabotPlugin);  
}



#endif // !_PARABOT_ROS_PLUGIN__