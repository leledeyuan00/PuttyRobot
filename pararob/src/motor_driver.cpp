#include "pararob/motor_driver.h"

#define MOTOR_PORT 0

unsigned char motor_data1_[10]={0xff,0xff,0xff,0x01,0x05,0xf3,0x86};
unsigned char motor_data2_[10]={0xff,0xff,0xff,0x02,0x05,0xf3,0x86};
unsigned char motor_data3_[10]={0xff,0xff,0xff,0x03,0x05,0xf3,0x86};

Motor_driver::Motor_driver(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    motor_driver_init();
    motor_ser_init(&motor_ser_, MOTOR_PORT);
}

void Motor_driver::motor_driver_init()
{
   motor_sub_  = nh_.subscribe("motor_topic", 10, &Motor_driver::drivercallback,this); 
}

void Motor_driver::drivercallback(const pararob::motor &msg)
{
    motor_msg_[0] = msg.motor1 *100 * MOTOR_RES / MOTOR_STROKE;
    motor_msg_[1] = msg.motor2 *100 * MOTOR_RES / MOTOR_STROKE;
    motor_msg_[2] = msg.motor3 *100 * MOTOR_RES / MOTOR_STROKE;
    ROS_INFO("Motor cmd is 1:[%d],2:[%d],3:[%d]",motor_msg_[0],motor_msg_[1],motor_msg_[2]);
    motor_write(motor_msg_);
}

int Motor_driver::motor_ser_init(serial::Serial* ser, int port)
{
    try 
    {
    //设置串口属性，并打开串口
        std_msgs::String msg;
        std::stringstream ss;
        ss << "/dev/ttyUSB" << port;
        msg.data = ss.str();

        // ROS_INFO("Open Port is %s", ss.str());
        
        ser->setPort(msg.data.c_str()); // （※）
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
    ROS_INFO("check_sum:%x",last);
    return last;
}

void Motor_driver::motor_write(unsigned short* msg)
{

    motor_data1_[7] = msg[0] & 0x00ff;
    motor_data1_[8] = (msg[0]>>8)& 0x00ff;
    motor_data1_[9] = motor_check_sum(motor_data1_);

    motor_data2_[7] = msg[1] & 0x00ff;
    motor_data2_[8] = (msg[1] >>8)& 0x00ff;
    motor_data2_[9] = motor_check_sum(motor_data2_);

    motor_data3_[7] = msg[2] & 0x00ff;
    motor_data3_[8] = (msg[2] >>8)& 0x00ff;
    motor_data3_[9] = motor_check_sum(motor_data3_);

    motor_ser_.write(motor_data1_,10);  //ROS串口写数据(※) 
    ros::Duration(0.001).sleep(); // 最低延迟时间 0.02s
    motor_ser_.write(motor_data2_,10);
    ros::Duration(0.001).sleep();
    motor_ser_.write(motor_data3_,10);
    ros::Duration(0.001).sleep();
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "Motor_driver");
    ros::NodeHandle nh;

    Motor_driver motor_driver(&nh);
    ros::spin();

    return 0;
}