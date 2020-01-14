#include "pararob/laser_talker.h"

static unsigned char laser_cmd[4] = {0x81, 0x04, 0x41, 0x44};

Laser_talker::Laser_talker(ros::NodeHandle* nodehandle):nh_(*nodehandle) // 构造函数
{ 
    laser_init();

    laser_ser_init(&laser1_ser_ , LASER_PORT);
    laser_ser_init(&laser2_ser_ , LASER_PORT+2);
    laser_ser_init(&laser3_ser_ , LASER_PORT+1);
}

void Laser_talker::laser_init(){
    laser_pub_ = nh_.advertise<pararob::sensor>("laser_topic",1000);
}

int Laser_talker::laser_ser_init(serial::Serial* ser,int port)
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
        ser->setBaudrate(38400); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser->setTimeout(to); 
        ser->open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to Open Laser Sensor Serial Port %d",port); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser->isOpen()) 
    { 
        ROS_INFO("Laser Sensor Serial Port%d Initialized",port); 
    } 
    else 
    { 
        return -1; 
    } 
    return 0;
}

double Laser_talker::fetch_data(serial::Serial* ser)
{
    if(ser->available())
    { 
        /* 从激光传感器 1 的串口读取反馈电报（十六进制字符串） */
        std_msgs::String result;               
        result.data = ser->read(ser->available());
        int len = result.data.length();    //获取数组长度
        unsigned char final_result[len];   //存放最终数据
        //该循环用于将负数转化为正，因为默认接收的格式为有符号char,超出127的数都会变为负数，
        //因此对于负数应加上256，再赋值给新定义的无符号char型, 最后得到的数据存于final_result[]数组中               
        for(int i = 0; i<len; i++)        
        {                                  
            if(result.data[i]<0)
            {           
                final_result[i] = result.data[i] + 256; 
            }
            else
            {
                final_result[i] = result.data[i];
            }
        }
        // ROS_INFO("Reading1: ");
        // for(int i = 0; i<len; i++)
        // {
        //     ROS_INFO("%x ",final_result[i]); 
        // }
        // ROS_INFO("len = %d",len);

        /* 提取反馈电报中的两个参数字节（第４、５）的低６位（字符位0~11 = 距离值0~4095 ；
        　　0对应测量范围最大值，4095对应测量范围最小值)，组成一个新的两字节十六进制数　*/
        unsigned int dist_hex;
        double dist_dec;
        unsigned int dist_high = 0x00FF;
        unsigned int dist_low =  0x00FF;
        dist_high &= final_result[3];
        dist_high &= 0x003F;
        dist_high <<= 6;
        dist_low &= final_result[4];
        dist_low &= 0x003F;
        dist_hex = (dist_high | dist_low) & 0x0FFF;
        if (dist_high!=0) // out of limit
        {
            /* code for True */
           dist_dec = 300.0 - ((220.0 * dist_hex) / 4095);         
        }
        else
        {
            dist_dec = -1;
        }        
        // ROS_INFO("Laser Distance : %f\n", dist_dec);
        return dist_dec;
    }
}

void Laser_talker::run()
{
    ros::Rate loop_rate(250);
    while (ros::ok())
    {
        /* code for loop body */
        laser_data_.state = 0;
        laser_data_.sensor1 = fetch_data(&laser1_ser_);
        if (laser_data_.sensor1<80 || laser_data_.sensor1>300){
            laser_data_.state |= ((unsigned short)0x01)<<0;
        }
        else{
            laser_data_.state &= ~(((unsigned short)0x01)<<0);
        }
        
        laser_data_.sensor2 = fetch_data(&laser2_ser_);
        if (laser_data_.sensor2<80 || laser_data_.sensor2>300){
            laser_data_.state |= ((unsigned short)0x01)<<4;
        }
        else{
            laser_data_.state &= ~(((unsigned short)0x01)<<4);
        } 
        laser_data_.sensor3 = fetch_data(&laser3_ser_);
        if (laser_data_.sensor3<80 || laser_data_.sensor3>300){
            laser_data_.state |= ((unsigned short)0x01)<<8;
        }
        else{
            laser_data_.state &= ~(((unsigned short)0x01)<<8);
        }


        laser_pub_.publish(laser_data_);

        laser1_ser_.write(laser_cmd,4);  //写数据(※)     
        laser2_ser_.write(laser_cmd,4);
        laser3_ser_.write(laser_cmd,4);

        // ros::Duration(0.02).sleep();
    
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }    
}


int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "laser_talker");
    ros::NodeHandle nh;

    Laser_talker laser_talker(&nh);
    laser_talker.run();

    return 0;
}