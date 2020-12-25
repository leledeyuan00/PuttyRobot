#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

/* privita include */
#include "ppr_msgs/encoder.h"
#include "ppr_msgs/setImMode.h"
#include "ppr_msgs/setPositionMode.h"
#include "ppr_msgs/setStiffDamp.h"
#include "ppr_msgs/setStmPosition.h"

#define RECEIVE_LENGTH  8
#define SEND_LENGTH     6
#define COM_K_MAX		10.0f
#define COM_D_MAX		1.0f
#define COM_RES_MAX		32767.0f
#define COM_RES_MAX_2	255.0f
#define COM_ANGLE_MAX   180.0f
#define COM_FORCE_MAX   2.0f

class STM32
{
public:
    STM32(ros::NodeHandle* nodehandle);

    void run();

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher stm_pub_;

    ros::ServiceServer set_position_mode_;
    ros::ServiceServer set_im_mode_;
    ros::ServiceServer set_stiff_damp_;
    ros::ServiceServer set_des_position_;
    
    

    serial::Serial stm_ser_;

    ppr_msgs::encoder encoder_data_;

    void stm_init();
    int encoder_ser_init(serial::Serial* ser, int port);
    int fetch_data(serial::Serial* ser);

    bool handle_set_im_mode(ppr_msgs::setImMode::Request  &req,
                            ppr_msgs::setImMode::Response &res);
    bool handle_set_position_mode(ppr_msgs::setPositionMode::Request  &req,
                                  ppr_msgs::setPositionMode::Response &res);
    bool handle_set_stiff_damp(ppr_msgs::setStiffDamp::Request  &req,
                               ppr_msgs::setStiffDamp::Response &res);
    bool handle_set_des_position(ppr_msgs::setStmPosition::Request  &req,
                                 ppr_msgs::setStmPosition::Response &res);
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    uint8_t crc_cal(uint8_t *buf, uint8_t size);

    typedef enum
    {
        GB_SET = 0X00,
        GB_POSE,				//rx
        GB_ANGLED, 			//rx
        GB_KD_DESIRED		//rx
    }ROS_CMD;

    typedef enum
    {
        GB_STOP = 0x00,
        GB_START,
        GB_PAUSE,
        GB_RESUME,
        GB_RESTART,
    }MAIN_MODE;			//high 8-bits of data

    typedef enum
    {	
        GB_POSITION = 0x00,
        GB_IMPADENCE,
        GB_ZEROFORCE
    }SUB_MODE;			//low 8-bits of data

    typedef struct
    {
        MAIN_MODE main_mode;
        SUB_MODE sub_mode;
    }SYS_CMD;
};
