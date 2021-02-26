#include "stm32_hw/stm32.h"

STM32::STM32(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    stm_init();

    int getparam;
    nh_.getParam("/stm32_hw/stm32_port",getparam);

    const int STM32_PORT = getparam;

    encoder_ser_init(&stm_ser_, STM32_PORT);
}

void STM32::stm_init(){
    stm_pub_ = nh_.advertise<ppr_msgs::encoder>("stm_topic",1000);
    set_im_mode_ = nh_.advertiseService("setImMode", &STM32::handle_set_im_mode,this);
    set_position_mode_ = nh_.advertiseService("setPositionMode", &STM32::handle_set_position_mode,this);
    set_stiff_damp_ = nh_.advertiseService("setStiffDamp",  &STM32::handle_set_stiff_damp,this);
    set_des_position_ = nh_.advertiseService("setStmPosition", &STM32::handle_set_des_position,this);
}

int STM32::encoder_ser_init(serial::Serial* ser,int port)
{
    try 
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "/dev/ttyUSB" << port;
        msg.data = ss.str();
        
        ser->setPort(msg.data.c_str());
        ser->setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser->setTimeout(to); 
        ser->open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to Open STM Serial Port %d",port); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser->isOpen()) 
    { 
        ROS_INFO("STM  Serial Port%d Initialized",port); 
    } 
    else 
    { 
        return -1; 
    } 
    return 0;
}

int STM32::fetch_data(serial::Serial* ser)
{
    int receive_error = 0;
    if(ser->available()&& ser->waitReadable ())
    { 
        std::vector<uint8_t> result2;
        const int len = ser->read(result2,RECEIVE_LENGTH*2);   
        unsigned char final_result[len];     
        for (size_t i=0; i< result2.size(); i++)
        {
            final_result[i] = result2.at(i);
        }
        /* find head of line */
        size_t pos_of_head = 0;
        for (size_t i = 0; i < len; i++)
        {
            if(final_result[i]==0xA5 && final_result[i+1] == 0x5A){
                pos_of_head = i;
                break;
            }
            else if (i == (len -1)) // cannot find
            {
                receive_error |= 1 <<0;
            }
        }
        int16_t recieve_data,recieve_data2;
        uint8_t recieve_data3;
        uint8_t error_code, status;
        const float RES = 32767;
        

        /* fetch data from serial prot */
        if(final_result[pos_of_head + 7] == crc_cal(&final_result[pos_of_head + 0],RECEIVE_LENGTH-1))
        {
            
            recieve_data =  (int16_t)((final_result[pos_of_head + 2] << 8) |
                                    (final_result[pos_of_head + 3] << 0));

            recieve_data2 = (int16_t)((final_result[pos_of_head + 4] << 8) |
                                    (final_result[pos_of_head + 5] << 0));

            recieve_data3 = final_result[pos_of_head + 6];                    
        }
        else
        {
            receive_error |= 1 << 1;
            ROS_INFO("INput_crc is[%x], CALculated_crc is [%x], error in CRC calculate", final_result[pos_of_head + 7],crc_cal(&final_result[pos_of_head + 0],RECEIVE_LENGTH-1));
        }

        /* decode data */

        error_code = (recieve_data3 & 0xf0) >> 4;
        status = recieve_data3 & 0x0f;

        encoder_data_.angle = recieve_data * COM_ANGLE_MAX /RES;

        encoder_data_.force = recieve_data2 * COM_FORCE_MAX / RES;
        
        encoder_data_.error_code = error_code;

        encoder_data_.status = status;
        /* flush the input buffer */
        ser->flushInput();
    }
    else
    {
        receive_error |= 1 << 3;
    }
    return receive_error;
}

bool STM32::handle_set_im_mode(ppr_msgs::setImMode::Request  &req,
                            ppr_msgs::setImMode::Response &res)
{
    unsigned char send_data[6];

    send_data[0] = 0xA5;
    send_data[1] = 0x5A;
    send_data[2] = GB_SET;
    send_data[3] = GB_START;
    send_data[4] = GB_IMPADENCE;

    send_data[5] = crc_cal(send_data,SEND_LENGTH-1);

    stm_ser_.write(send_data,SEND_LENGTH);

    res.success = true;
    return true;
}
bool STM32::handle_set_position_mode(ppr_msgs::setPositionMode::Request  &req,
                                  ppr_msgs::setPositionMode::Response &res)
{
    unsigned char send_data[6];

    send_data[0] = 0xA5;
    send_data[1] = 0x5A;
    send_data[2] = GB_SET;
    send_data[3] = GB_START;
    send_data[4] = GB_POSITION;

    send_data[5] = crc_cal(send_data,SEND_LENGTH-1);

    stm_ser_.write(send_data,SEND_LENGTH);

    res.success = true;
    return true;
}
bool STM32::handle_set_stiff_damp(ppr_msgs::setStiffDamp::Request  &req,
                               ppr_msgs::setStiffDamp::Response &res)
{
    float k_im, d_im;
    unsigned char send_data[6];
    unsigned char k_im_char,d_im_char;

    k_im = req.k_im;
    d_im = req.d_im;

    k_im_char = (int16_t)((k_im  * COM_RES_MAX_2) / COM_K_MAX ) & 0x00ff;
    d_im_char = (int16_t)((d_im  * COM_RES_MAX_2) / COM_D_MAX ) & 0x00ff;

    send_data[0] = 0xA5;
    send_data[1] = 0x5A;
    send_data[2] = GB_KD_DESIRED;
    send_data[3] = k_im_char;
    send_data[4] = d_im_char;

    send_data[5] = crc_cal(send_data,SEND_LENGTH-1);

    stm_ser_.write(send_data,SEND_LENGTH);

    res.success = true;
    return true;
}

bool STM32::handle_set_des_position(ppr_msgs::setStmPosition::Request  &req,
                                    ppr_msgs::setStmPosition::Response &res)
{
    float des_pos = req.data;
    float tempf;
    int16_t tempi;
    unsigned char send_data[6];

    if(des_pos < COM_ANGLE_MAX && des_pos > -COM_ANGLE_MAX)
        tempf = des_pos;
    else
        tempf = COM_ANGLE_MAX - des_pos;
    tempi = (tempf / COM_ANGLE_MAX) * COM_RES_MAX;

    send_data[0] = 0xA5;
    send_data[1] = 0x5A;
    send_data[2] = GB_ANGLED;
    send_data[3] = (tempi>>8) & 0x00ff;
    send_data[4] =  tempi & 0x00ff;

    send_data[5] = crc_cal(send_data,SEND_LENGTH-1);
    // stm_ser_.flushInput();
    // stm_ser_.flushOutput();
    stm_ser_.write(send_data,SEND_LENGTH);
    ROS_INFO("send data is[%02x %02x %02x %02x %02x %02x]",send_data[0],send_data[1],send_data[2],send_data[3],send_data[4],send_data[5]);
    res.success = true;
    return true;
}

uint8_t STM32::crc8_update(uint8_t crc, uint8_t data)
{
	uint8_t i;
	crc = crc ^ data;
	for (i = 0; i < 8; i++)
	{
		if(crc & 0x01)
		{
			crc = (crc >> 1) ^ 0x80;
		}
		else
		{
			crc >>= 1;
		}
	}
	return crc;
}

uint8_t STM32::crc_cal(uint8_t *buf, uint8_t size)
{
	uint8_t temp_crc = 0x00;
	for(uint8_t i=0; i<size; i++)
	{
		temp_crc = crc8_update(temp_crc, buf[i]);
	}
	return temp_crc;
}

void STM32::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        /* code for loop body */
        if(fetch_data(&stm_ser_) == 0){
            stm_pub_.publish(encoder_data_);
        }
    
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