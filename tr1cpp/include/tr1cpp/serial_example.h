//#ifndef SERIAL_EXAMPLE_H
//#define SERIAL_EXAMPLE_H
//extern "C" {
//#include "er_ti_f28069m_drv8305/er_buffer.h"
//#include "er_ti_f28069m_drv8305/er_command.h"
//#include "er_ti_f28069m_drv8305/er_msg.h"
//#include "er_ti_f28069m_drv8305/er_registers.h"
//}
//#include <fcntl.h>
//#include <pthread.h>
//#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <string>
//#include <termios.h>
//#include <time.h>
//
//class Serial
//
//{
//public:
//    Serial(ros::NodeHandle &nh, uint16_t source_id, uint16_t node_id, std::string serial_port);
//    ~Serial();
//    bool serial_init(std::string serial_port);
//    int serial_write(const std::string &data);
//    const float &serial_read();
//    std::string get_pos_cmd(uint16_t source_id, uint16_t node_id);
//    std::string set_pos_cmd(uint16_t source_id, uint16_t node_id, float position);
//    std::string ping_cmd(uint16_t source_id, uint16_t node_id);
//    void ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port);
//    std::string hexStr(unsigned char *data, int len);
//    void write_callback(const std_msgs::String::ConstPtr &msg);
//
//protected:
//    int serial_port_fd;
//
//    int no_read_bytes = 0;
//    std::vector<uint8_t> serial_read_buffer;
//    std::vector<uint16_t> ti_read_buffer;
//
//    int no_written_bytes = 0;
//
//    char *ping_byte_array, *get_pos_byte_array, *set_pos_byte_array; //= "";
//    std::string ping_byte_str, get_pos_byte_str, set_pos_byte_str;
//    uint16_t ping_cmd_len, get_pos_cmd_len, set_pos_cmd_len;
//    float current_position;
//    float desired_pose = 0.0;
//
//    ros::NodeHandle nh_;
//    ros::Subscriber write_sub;
//    ros::Publisher read_pub;
//    ros::Publisher write_pub_str;
//
//    volatile unsigned char _RX_SCI_buf[1024]; // buffer RX
//    volatile uint16_t _RX_SCI_ptr = 0;
//    volatile uint16_t __flag_buffer_rx_is_changed = 0;
//    volatile uint16_t _node_id = 1;
//    volatile ER_Msg gMsgCommand = ER_Msg_INIT;
//    volatile ER_Msg gMsgResponse = ER_Msg_INIT;
//};
//#endif
#ifndef SERIAL_EXAMPLE_H
#define SERIAL_EXAMPLE_H
extern "C" {
#include "er_ti_f28069m_drv8305/er_buffer.h"
#include "er_ti_f28069m_drv8305/er_command.h"
#include "er_ti_f28069m_drv8305/er_msg.h"
#include "er_ti_f28069m_drv8305/er_registers.h"
}
#include "geometry_msgs/WrenchStamped.h"
#include <fcntl.h>
#include <pthread.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <string>
#include <termios.h>
#include <time.h>

class Serial

{
public:
    Serial(ros::NodeHandle &nh, uint16_t source_id, uint16_t node_id, std::string serial_port);
    ~Serial();
    bool serial_init(std::string serial_port);
    int serial_write(const std::string &data);
    void serial_read(std::vector<uint16_t> *tactile_serial_read, float *current_pose);
    void serial_read(float *current_pose);
    void serial_read(std::vector<uint16_t> *tactile_serial_read);
    void serial_read();

    void check_response_type(volatile ER_Msg *_msg,
                             std::vector<uint16_t> *serial_read_tactile,
                             float *current_pose);
    void check_response_type(volatile ER_Msg *_msg, float *current_pose);
    void check_response_type(volatile ER_Msg *_msg, std::vector<uint16_t> *serial_read_tactile);
    void check_response_type(volatile ER_Msg *_msg);

    std::string get_pos_cmd(uint16_t source_id, uint16_t node_id);
    std::string set_pos_cmd(uint16_t source_id, uint16_t node_id, float position);
    std::string ping_cmd(uint16_t source_id, uint16_t node_id);

    std::string tactile_pcb1_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_pcb2_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_finger1_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_pcb3_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_pcb4_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_finger2_get_median_cmd(uint16_t source_id, uint16_t node_id);
    std::string tactile_gripper1_get_median_cmd(uint16_t source_id, uint16_t node_id);

    void ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port);
    std::string hexStr(unsigned char *data, int len);
    void set_pose_callback(const std_msgs::Float64::ConstPtr &msg);
    // void get_pose_callback(const std_msgs::Float64::ConstPtr &msg);

protected:
    int serial_port_fd;

    int no_read_bytes = 0;
    std::vector<uint8_t> serial_read_buffer;
    std::vector<uint16_t> ti_read_buffer;

    int no_written_bytes = 0;

    // BLDC Module
    char *ping_byte_array, *get_pos_byte_array, *set_pos_byte_array; //= "";
    std::string ping_byte_str, get_pos_byte_str, set_pos_byte_str;
    uint16_t ping_cmd_len, get_pos_cmd_len, set_pos_cmd_len;
    float current_position;
    float desired_pose = 0.0;

    // Tactile Module
    char *pcb1_byte_array, *pcb2_byte_array, *finger1_byte_array, *pcb3_byte_array,
        *pcb4_byte_array, *finger2_byte_array, *gripper1_byte_array; //= "";
    std::string pcb1_byte_str, pcb2_byte_str, finger1_byte_str, pcb3_byte_str, pcb4_byte_str,
        finger2_byte_str, griper1_byte_str;
    uint16_t pcb1_cmd_len, pcb2_cmd_len, fnger1_cmd_len, pcb3_cmd_len, pcb4_cmd_len, fnger2_cmd_len,
        gripper1_cmd_len;
    std::vector<uint16_t> tactile_serial_read;
    int NO_OF_CHANNELS = 9;
    int NO_OF_PCBS = 2;
    int NO_OF_FINGERS = 2;

    ros::NodeHandle nh_;
    ros::Subscriber write_sub;

    ros::Publisher read_pub;
    std_msgs::Float64 get_pose_msg;
    std_msgs::Float64MultiArray pcb1_voltages_msg;

    ros::Publisher write_pub_str;

    ros::Publisher tactile_force_pub;

    ros::Publisher pcb1_sensors_voltage_pub;

    //ros::Publisher array1_pub;
    //ros::Publisher array2_pub;
    //ros::Publisher array3_pub;
    //ros::Publisher array4_pub;
    //ros::Publisher array5_pub;
    //ros::Publisher array6_pub;
    //ros::Publisher array7_pub;
    //ros::Publisher array8_pub;
    //ros::Publisher array9_pub;

    volatile unsigned char _RX_SCI_buf[1024]; // buffer RX
    volatile uint16_t _RX_SCI_ptr = 0;
    volatile uint16_t __flag_buffer_rx_is_changed = 0;
    volatile uint16_t _node_id = 1;
    volatile ER_Msg gMsgCommand = ER_Msg_INIT;
    volatile ER_Msg gMsgResponse = ER_Msg_INIT;
};

#endif
