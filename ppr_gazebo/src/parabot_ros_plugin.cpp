#include "parabot_ros_plugin.h"
namespace gazebo
{
  void ParabotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    ROS_INFO("Loading parabot_ros_plugin");
    // Save pointers to the model
    model_ = _model;
    sdf_ = _sdf;

    // Error message if the model couldn't be found
    if (!model_)
    {
      ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
      return;
    }

    // Check that ROS has been initialized
    if(!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("gazebo_ros_control","A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Get the gazebo control period
    ros::Duration gazebo_period(model_->GetWorld()->Physics()->GetMaxStepSize());

    // Decide the plugin control period
    if(sdf_->HasElement("controlPeriod"))
    {
      control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

      // Check the period against the simulation period
      if( control_period_ < gazebo_period )
      {
        ROS_ERROR_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
          <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
      }
      else if( control_period_ > gazebo_period )
      {
        ROS_WARN_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
          <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
      }
    }
    else
    {
      control_period_ = gazebo_period;
      ROS_DEBUG_STREAM_NAMED("gazebo_ros_control","Control period not found in URDF/SDF, defaulting to Gazebo period of "
        << control_period_);
    }
    // Get namespace for nodehandle
    /* element parse */
    std::string robot_namespace_;
    if(sdf_->HasElement("robotNamespace"))
    {
      robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
    }
    else
    {
      robot_namespace_ = model_->GetName(); // default
    }

    rosNode = ros::NodeHandle(robot_namespace_);
    std::vector<std::string> joint_names_;
    if(sdf_->HasElement("jointName"))
    {
      joint_parse_ = sdf_->GetElement("jointName")->Get<std::string>();
      std::stringstream joint_parse_ss(joint_parse_);
      std::istream_iterator<std::string> begin(joint_parse_ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vjoint_name(begin,end);
      joint_names_ = vjoint_name;
      ROS_INFO_STREAM_NAMED("load joints","joints name is "+ joint_parse_);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("load joints","joints is NULL");
      return;
    }
    // Initialize the emergency stop code.
    e_stop_active_ = false;
    last_e_stop_active_ = false;
    if (sdf_->HasElement("eStopTopic"))
    {
      const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
      e_stop_sub_ = rosNode.subscribe(e_stop_topic, 1, &ParabotPlugin::eStopCB, this);
    }

    // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      parabot_hw_sim_.reset(
        new parabot_hw_sim(model_,joint_names_,robot_namespace_));
      if(!parabot_hw_sim_->initSim())
      {
        ROS_FATAL_NAMED("gazebo_ros_control","Could not initialize robot simulation interface");
        return;
      }

      // Create the controller manager
      ROS_INFO_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
      controller_manager_.reset
        (new controller_manager::ControllerManager(parabot_hw_sim_.get(), rosNode));

      // Listen to the update event. This event is broadcast every simulation iteration.
      update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
        (boost::bind(&ParabotPlugin::Update, this));

    }
    catch(pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM_NAMED("ros_control_plugin","Failed to create robot simulation interface loader: "<<ex.what());
    }
  } 

  // Called by the world update start event
  void ParabotPlugin::Update()
  {
    common::Time gz_time_now = model_->GetWorld()->SimTime();

    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

    parabot_hw_sim_->eStopActive(e_stop_active_); // by a ros sub topic 
    if (sim_period >= control_period_)
    {
      // Store this simulation time
      last_update_sim_time_ros_ = sim_time_ros;

      // Update the robot simulation with the state of the gazebo model
      parabot_hw_sim_-> readSim(sim_time_ros, sim_period);

      // Compute the controller commands
      bool reset_ctrlrs;
      if(e_stop_active_)
      {
        reset_ctrlrs = false;
        last_e_stop_active_=true;
      }
      else
      {
        if(last_e_stop_active_)
        {
          reset_ctrlrs = true;
          last_e_stop_active_ = false;
        } 
        else
        {
          reset_ctrlrs = false;
        }
      }
      controller_manager_->update(sim_time_ros,sim_period,reset_ctrlrs);
    }

    // Update the gazebo model with the result of the controller computation
    parabot_hw_sim_ -> writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
    last_write_sim_time_ros_ = sim_time_ros;
  }

  // Called on world reset
  void ParabotPlugin::Reset()
  {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros_ = ros::Time();
    last_write_sim_time_ros_ = ros::Time();
  }

  // Emergency stop callback
  void ParabotPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
  {
    e_stop_active_ = e_stop_active->data;
  }


}