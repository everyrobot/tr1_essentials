#ifndef SERIAL_EXAMPLE_H
#define SERIAL_EXAMPLE_H
extern "C" {
#include "er_ti_f28069m_drv8305/er_buffer.h"
#include "er_ti_f28069m_drv8305/er_msg.h"
#include "er_ti_f28069m_drv8305/er_registers.h"
#include "er_ti_f28069m_drv8305/er_command.h"
}
#include <array>
//#include <bits/stdc++.h>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdlib.h>
#include <string>
#include <time.h>
class Serial

{
public:
    //serial::Serial ser;
    Serial(ros::NodeHandle &nh,
           uint16_t source_id,
           uint16_t node_id,
           float &current_pos,
           float desired_pos = 0);
    ~Serial();
    bool init();
    std::string hexStr(unsigned char *data, int len);
    void write_callback(const std_msgs::String::ConstPtr &msg);
    std::string ping_cmd(uint16_t source_id, uint16_t node_id);
    std::string get_pos_cmd(uint16_t source_id, uint16_t node_id);
    std::string set_pos_cmd(uint16_t source_id, uint16_t node_id, float position);
    void serial_read();
    //void serial_write(std::string serial_write_ros);
    bool get_pos_resp(float &pos);
    bool set_pos_resp();
    void get_pos_read(float &pos, double elapsed_time);
    void set_pos_read(double elapsed_time);
    void serial_write(std::string cmd);
    void ros_loop(uint16_t source_id, uint16_t node_id, float &current_pos, float desired_pos = 0);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber write_sub;
    ros::Publisher read_pub;
    ros::Publisher write_pub_str;
    serial::Serial ser;

    char *byte_array_ping, *byte_array_get_pos, *byte_array_set_pos; //= "";
    std_msgs::String str_ros_ping, str_ros_get_pos, str_ros_set_pos;
    std::string data, serial_read_hexstr, byte;
    uint8_t *serial_read_char_array;
    uint16_t msg_ping_len, cmd_get_pos_len, cmd_set_pos_len;
    //char *data;
    uint16_t flags;
    volatile unsigned char _RX_SCI_buf[1024]; // buffer RX
    volatile uint16_t _RX_SCI_ptr = 0;
    volatile uint16_t __flag_buffer_rx_is_changed = 0;
    volatile ER_Msg gMsgCommand = ER_Msg_INIT;
    volatile ER_Msg gMsgResponse = ER_Msg_INIT;
    volatile uint16_t _node_id = 1;
    float current_position;
    //ros::Time start_time;
    clock_t start, end;
};

#endif



