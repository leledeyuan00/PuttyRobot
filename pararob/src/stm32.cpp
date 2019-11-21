#include "pararob/stm32.h"

#define STM_PORT 4

STM32::STM32(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    stm_init();

    encoder_ser_init(&stm_ser_, STM_PORT);
}

void STM32::stm_init(){
    stm_pub_ = nh_.advertise<pararob::encoder>("stm_topic",1000);
}

int STM32::encoder_ser_init(serial::Serial* ser,int port)
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

float STM32::fetch_data(serial::Serial* ser)
{
    if(ser->available()&& ser->waitReadable ())
    { 
        // ser-> flushInput();
        std_msgs::String result;               
        result.data = ser->read(ser->available());
        int len = result.data.length();    //获取数组长度
        unsigned char final_result[len];   //存放最终数据     
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

        unsigned int recieve_data;
        float res = 131071;
        

        for (size_t i = 0; i < len; i++)
        {
            /* code for loop body */
            if(final_result[i] == 0xA5 && final_result[i+1]== 0x5A )
            {
                unsigned char temp;
                temp = 0;
                for (size_t j = 0; j < 5; j++)
                {
                    /* code for loop body */
                    temp = temp ^ final_result[i+j];
                }
                if(temp == final_result[i+5]);
                {
                    recieve_data =  (((unsigned int)final_result[i+2]&0x01) << 16) | 
                                    (((unsigned int)final_result[i+3]&0xff) << 8) |
                                    (((unsigned int)final_result[i+4]&0xff) << 0);
                    break;
                }
            }
        }
        if(recieve_data< ((res+1)/2))
        {
            encoder_data_.angle = recieve_data *360 /(res);
            ROS_INFO("angle = %f",encoder_data_.angle);
        }
        else if(recieve_data >= ((res+1)/2) && recieve_data<=res)
        {
            encoder_data_.angle = (recieve_data - res) *360 /(res);
            ROS_INFO("angle = %f",encoder_data_.angle);
        }   
    }
}

void STM32::run()
{
    ros::Rate loop_rate(250);
    while (ros::ok())
    {
        /* code for loop body */
        fetch_data(&stm_ser_);
        stm_pub_.publish(encoder_data_);
    
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }    
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "stm_talker");
    ros::NodeHandle nh;

    STM32 stm_talker(&nh);
    stm_talker.run();

    return 0;
}