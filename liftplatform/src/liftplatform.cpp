#include "liftplatform/liftplatform.h"

liftplatform::liftplatform(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    int getparam;
    nh_.getParam("/liftplatform/liftplatform_port", getparam);
    const int PORT = getparam;
    ROS_INFO("lift port is %d",PORT);
    // int  PORT = 3;
    encoder_ser_init(&stm_ser_, PORT);

    stm_init();

}

void liftplatform::stm_init()
{
    stm_pub_ = nh_.advertise<ppr_msgs::encoder>("liftplat_topic",1000);
    set_mode_ = nh_.advertiseService("/LiftPlat/setMode", &liftplatform::handle_set_mode,this);
    set_des_speed_ = nh_.advertiseService("/LiftPlat/setSpeed", &liftplatform::handle_set_des_speed,this);
}

int liftplatform::encoder_ser_init(serial::Serial* ser,int port)
{
    try 
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "/dev/ttyUSB" << port;
        msg.data = ss.str();
        
        ser->setPort(msg.data.c_str());
        ser->setBaudrate(19200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser->setTimeout(to); 
        ser->open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR("Unable to Open Serial Port %d",port); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser->isOpen()) 
    { 
        ROS_INFO(" Serial Port%d Initialized",port); 
    } 
    else 
    { 
        return -1; 
    } 
    return 0;
}

/*CRC 直接计算法*/
// int liftplatform::fetch_data(serial::Serial* ser) // 計算CRC相關參數
// {
//     int receive_error = 0;
//     if(ser->available()&& ser->waitReadable ())
//     { 
//         std::vector<uint8_t> result2;
//         const int len = ser->read(result2,RECEIVE_LENGTH*2);   //RECEIVE_LENGTH長度如何確定
//                                                                                                                          //有的是9字節 ，有的是14字節
//         unsigned char final_result[len];    // final_result 是什麼？
//         for (size_t i=0; i < result2.size(); i++)
//         {
//             final_result[i] = result2.at(i);
//         }
//         /* find head of line */
//         size_t pos_of_head;
//         for (size_t i = 0; i < len; i++)
//         {cur_position
//                 break;
//             }
//             else if (i == (len -1)) // cannot find
//             {
//                 receive_error |= 1 <<0; //receive_error 是什麼？
//             }
//         }
//         int16_t recieve_data,recieve_data2;
//         uint8_t recieve_data3;cur_position
//         uint8_t error_code, status;
//         const float RES = 32767;
        
//         /* fetch data from serial prot */
//         if(final_result[pos_of_head + 7] == cal_crc_table(&final_result[pos_of_head + 0],RECEIVE_LENGTH-1))
//         {
//             recieve_data =  (int16_t)((final_result[pos_of_head + 2] << 8) |
//                                     (final_result[pos_of_head + 3] << 0));

//             recieve_data2 = (int16_t)((final_result[pos_of_head + 4] << 8) |
//                                     (final_result[pos_of_head + 5] << 0));

//             recieve_data3 = final_result[pos_of_head + 6];                    
//         }
//         else
//         {
//             receive_error |= 1 << 1;
//             ROS_INFO("INput_crc is[%x], CALculated_crc is [%x], error in CRC calculate", final_result[pos_of_head + 7],crc_cal(&final_result[pos_of_head + 0],RECEIVE_LENGTH-1));
//         }

//         /* decode data */

//         error_code = (recieve_data3 & 0xf0) >> 4;
//         status = recieve_data3 & 0x0f;

//         // encoder_data_.angle = recieve_data * COM_ANGLE_MAX /RES;

//         // encoder_data_.force = - recieve_data2 * COM_FORCE_MAX / RES;
        
//         encoder_data_.error_code = error_code;

//         encoder_data_.status = status;
//         /* flush the input buffer */
//         ser->flushInput();
//     }
//     else
//     {
//         receive_error |= 1 << 3; //51單片機的檢驗方式
//     }
//     return receive_error;
// }

/*service設置使能*/
bool liftplatform::handle_set_mode(liftplat_msgs::setEnableMod::Request &req, 
                            liftplat_msgs::setEnableMod::Response &res)
{
    unsigned char send_data[SEND_LENGTH];
    unsigned int receive_crc_data;
    send_data[0] = 0x01;
    send_data[1] = 0x06;
    send_data[2] = 0x00;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = 0x01;
 
    receive_crc_data = cal_crc_table(send_data,SEND_LENGTH-2);

    send_data[6] = 0x00ff & (receive_crc_data >> 8);
    send_data[7] = 0x00ff & (receive_crc_data );


    stm_ser_.write(send_data,SEND_LENGTH ); // write the crc number ???

    ROS_INFO("send data is[%02x %02x %02x %02x %02x %02x %02x %02x]",
                                    send_data[0],send_data[1],send_data[2],send_data[3],send_data[4],send_data[5],send_data[6],send_data[7]);

    res.success = true;
    return true;
}
/*service設置目標轉速*/
bool liftplatform::handle_set_des_speed(liftplat_msgs::setLiftplatPosition::Request &req, 
                            liftplat_msgs::setLiftplatPosition::Response &res)
{
    int des_speed = req.speed;
    int des_position = req.position;
    nh_.setParam("Des_Height_LiftPlat", des_position);
    int cur_position;
    nh_.getParam("iDis_LiftplatHeight_Read", cur_position);

    ROS_INFO("cur_position: %d", cur_position);
    ROS_INFO("des_position: %d", des_position);

    unsigned char send_data[SEND_LENGTH];
    unsigned int receive_crc_data;

    send_data[0] = 0x01;
    send_data[1] = 0x06;
    send_data[2] = 0x00;
    send_data[3] = 0x02;
    send_data[4] = 0xff & (des_speed >> 8);
    send_data[5] = 0xff & (des_speed);

    receive_crc_data = cal_crc_table(send_data,SEND_LENGTH-2);

    send_data[6] = 0x00ff & (receive_crc_data >> 8); 
    send_data[7] = 0x00ff & (receive_crc_data );

    stm_ser_.flushInput();
    stm_ser_.flushOutput();
    stm_ser_.write(send_data,SEND_LENGTH );  // write the crc number ???
    ROS_INFO("send data is[%02x %02x %02x %02x %02x %02x %02x %02x]",
                                    send_data[0],send_data[1],send_data[2],send_data[3],send_data[4],send_data[5],send_data[6],send_data[7]);
    
    res.success = true;
    return true;
}

// 电机停转
void liftplatform::handle_set_zero_speed()
{
    int des_speed = 0;
    
    unsigned char send_data[SEND_LENGTH];
    unsigned int receive_crc_data;

    send_data[0] = 0x01;
    send_data[1] = 0x06;
    send_data[2] = 0x00;
    send_data[3] = 0x02;
    send_data[4] = 0xff & (des_speed >> 8);
    send_data[5] = 0xff & (des_speed);

    receive_crc_data = cal_crc_table(send_data,SEND_LENGTH-2);

    send_data[6] = 0x00ff & (receive_crc_data >> 8); 
    send_data[7] = 0x00ff & (receive_crc_data );

    stm_ser_.flushInput();
    stm_ser_.flushOutput();
    stm_ser_.write(send_data,SEND_LENGTH ); 

    return ;
}

//位置控制
void liftplatform::position_control(liftplatform & liftplat)
{
    int cur_position;
    int des_position;
    int error = 10;
    nh_.getParam("iDis_LiftplatHeight_Read", cur_position);
    nh_.getParam("Des_Height_LiftPlat", des_position);
    if(abs(cur_position - des_position) < error){ // 设置升降平台终止条件
        ROS_INFO("current height: %d", cur_position);
        liftplat.handle_set_zero_speed();
    }
}

/*CRC 直接计算法实现*/
// uint8_t liftplatform::crc8_update(uint8_t crc, uint8_t data)
// {
// 	uint8_t i;
// 	crc = crc ^ data;
// 	for (i = 0; i < 8; i++)
// 	{
// 		if(crc & 0x01)
// 		{
// 			crc = (crc >> 1) ^ 0x8C;
// 		}
// 		else
// 		{
// 			crc >>= 1;
// 		}
// 	}
// 	return crc;
// }

// uint8_t liftplatform::crc_cal(uint8_t *buf, uint8_t size)
// {
// 	uint8_t temp_crc = 0x00;
// 	for(uint8_t i=0; i<size; i++)
// 	{
// 		temp_crc = crc8_update(temp_crc, buf[i]);
// 	}
// 	return temp_crc;
// }

/*CRC 查表法实现*/
unsigned int liftplatform::cal_crc_table(unsigned char *puchMsg, unsigned char usDataLen) 
{
    // unsigned char  temp_crc = 0x00;
    unsigned char uchCRCHi = 0xFF ; /* 高 CRC 字节初始化 */
    unsigned char uchCRCLo = 0xFF; /* 低 CRC 字节初始化 */
    unsigned uIndex ; /* CRC 循环中的索引 */
    while (usDataLen--)
    {
        uIndex = uchCRCHi ^ *puchMsg++ ; //计算 CRC，異或操作？
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}

void liftplatform::run(liftplatform & liftplat)
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {    
        ros::spinOnce(); 
        liftplat.position_control(liftplat);
        loop_rate.sleep(); 
    }    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "liftplat");
    ros::NodeHandle nh;

    liftplatform liftplat(&nh);    

    liftplat.run(liftplat);
 
    return 0;
}