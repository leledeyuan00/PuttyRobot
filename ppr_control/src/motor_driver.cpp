#include "motor_driver/motor_driver.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

unsigned char motor_cmd_all_[18] = {0xff,0xff,0xff,0x0fe,0x0D,0x73,0x86,0x02};

// unsigned char motor_force_[9] = {0xff,0xff,0xff,0x01,0x04,0x0f3,0x80,0x00};

Motor_driver::Motor_driver(ros::NodeHandle* nodehandle):
    nh_(*nodehandle),name_("motor_driver"),
    use_rosparam_joint_limits_(false),use_soft_limits_if_available_(false)
{
    /* fetch Serial port  */
    int getparam;
    nh_.getParam("motor_port",getparam);

    const int MOTOR_PORT = getparam;
    motor_ser_init(&motor_ser_, MOTOR_PORT);

    motor_driver_init();

    controller_manager_.reset(new controller_manager::ControllerManager(this,nh_));

    loop_hz_ = 125;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    my_control_loop_ = nh_.createTimer(update_freq,&Motor_driver::update,this);
}

void Motor_driver::motor_driver_init()
{
    motor_sub_  = nh_.subscribe("motor_topic", 10, &Motor_driver::drivercallback,this); 

    if (urdf_model_ == NULL)
      loadURDF(nh_, "robot_description");

    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, nh_, "joints", joint_names_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    MOTOR_NUM = joint_names_.size();

    joint_position_lower_limits_.resize(MOTOR_NUM, 0.0);
    joint_position_upper_limits_.resize(MOTOR_NUM, 0.0);
    joint_velocity_limits_.resize(MOTOR_NUM, 0.0);
    joint_effort_limits_.resize(MOTOR_NUM, 0.0);
    jnt_.resize(MOTOR_NUM);

    motor_msg_.resize(MOTOR_NUM);
    motor_msg2_.resize(MOTOR_NUM);

    /* for controller */
    for (size_t joint_id = 0; joint_id < joint_names_.size(); joint_id++)
    {
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names_[joint_id],&jnt_[joint_id].state.pos,&jnt_[joint_id].state.vel,&jnt_[joint_id].state.eft
        );
        jnt_st_int_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle_position(
            jnt_st_int_.getHandle(joint_names_[joint_id]),&jnt_[joint_id].cmd.pos
        );
        pos_jnt_int_.registerHandle(joint_handle_position);

        hardware_interface::JointHandle joint_handle_velocity(
            jnt_st_int_.getHandle(joint_names_[joint_id]),&jnt_[joint_id].cmd.vel
        );
        vel_jnt_int_.registerHandle(joint_handle_velocity);

        hardware_interface::JointHandle joint_handle_effort(
            jnt_st_int_.getHandle(joint_names_[joint_id]),&jnt_[joint_id].cmd.eft
        );
        eft_jnt_int_.registerHandle(joint_handle_effort);

        registerJointLimits(joint_handle_position,joint_handle_velocity,joint_handle_effort,joint_id);
    }
    registerInterface(&jnt_st_int_);
    registerInterface(&pos_jnt_int_);
    registerInterface(&vel_jnt_int_);
    registerInterface(&eft_jnt_int_);
}

void Motor_driver::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  std::shared_ptr<const urdf::Joint> urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);
  // Get main joint limits
  if (urdf_joint == NULL)
  {
    ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                            << joint_limits.min_position << ", "
                                                            << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                              << joint_limits.max_velocity << "]");
  }
  else
  {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
                            "position limit");
  }

  // Get limits from ROS param
  if (use_rosparam_joint_limits_)
  {
    if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                                      << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                                << " has rosparam velocity limit ["
                                                                << joint_limits.max_velocity << "]");
    }  // the else debug message provided internally by joint_limits_interface
  }

  // Get soft limits from URDF
  if (use_soft_limits_if_available_)
  {
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
    {
      has_soft_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint "
                             "limits");
    }
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }
  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  // Copy effort limits if available
  if (joint_limits.has_effort_limits)
  {
    joint_effort_limits_[joint_id] = joint_limits.max_effort;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                       joint_limits, soft_limits);
    pos_jnt_soft_limits_.registerHandle(soft_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                       joint_limits, soft_limits);
    vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
    const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                   soft_limits);
    eft_jnt_soft_limits_.registerHandle(soft_handle_effort);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
    pos_jnt_limit_int_.registerHandle(sat_handle_position);

    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
    vel_jnt_limit_int_.registerHandle(sat_handle_velocity);

    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    eft_jnt_limit_int_.registerHandle(sat_handle_effort);
  }
}

// control loop
void Motor_driver::update(const ros::TimerEvent& e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_ -> update(ros::Time::now(),elapsed_time_);
    write(elapsed_time_);
}

void Motor_driver::read()
{
    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        jnt_[i].state.pos = jnt_[i].cmd.pos; // just equal command because we do not need read motor position;
    }
}

void Motor_driver::write(ros::Duration elapsed_time)
{
    pos_jnt_limit_int_.enforceLimits(elapsed_time);

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        motor_msg2_[i] = jnt_[i].cmd.pos * 1000 * MOTOR_RES / MOTOR_STROKE;
    }

    motor_write(motor_msg2_);
}

void Motor_driver::drivercallback(const motor_driver::motor &msg)
{
    motor_msg_[0] = msg.motor1 *1000 * MOTOR_RES / MOTOR_STROKE;
    motor_msg_[1] = msg.motor2 *1000 * MOTOR_RES / MOTOR_STROKE;
    motor_msg_[2] = msg.motor3 *1000 * MOTOR_RES / MOTOR_STROKE;    
    motor_write(motor_msg_);
}

int Motor_driver::motor_ser_init(serial::Serial* ser,const int port)
{
    try 
    {
    //设置串口属性，并打开串口
        std::string msg;
        msg = {"/dev/ttyUSB" + std::to_string(port)};
        
        ser->setPort(msg); // （※）
        ser->setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser->setTimeout(to); 
        ser->open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to Open Motor Serial Port %d",port); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser->isOpen()) 
    { 
        ROS_INFO("Motor Serial Port%d Initialized",port); 
    } 
    else 
    { 
        return -1; 
    } 
    return 0;
}

unsigned char Motor_driver::motor_check_sum(unsigned char* data)
{
    unsigned char temp=0,last=0;
    for(int i=3;i<data[4]+4;i++) //i=3 for ignore msg head
    {
        temp += data[i];
    }
    last = ~temp;
    return last;
}

void Motor_driver::motor_write(std::vector<short unsigned int>& msg)
{
    /* for all motor once communicate */
    for (size_t i = 0; i < 3; i++)
    {
        motor_cmd_all_[8 + i*3] = i + 1;
        motor_cmd_all_[9 + i*3] = msg[i] & 0x00ff;
        motor_cmd_all_[10 + i*3]= (msg[i]>>8) & 0x00ff;
    }
    motor_cmd_all_[17] = motor_check_sum(motor_cmd_all_);
    motor_ser_.write(motor_cmd_all_,18);
}

void Motor_driver::run()
{
    while (ros::ok())
    {
        /* spin loop */
    }
    my_control_loop_.stop();
}

void Motor_driver::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;

  if (urdf_model_ == NULL){
    urdf_model_.reset(new urdf::Model());
  }
  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM( "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_STREAM( "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << param_name);
      nh.getParam(param_name, urdf_string);
    }
  
    if (!urdf_model_->initString(urdf_string))
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    else
      ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    usleep(100000);
  }
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "Motor_driver");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    Motor_driver motor_driver(&nh);
    motor_driver.run();

    return 0;
}