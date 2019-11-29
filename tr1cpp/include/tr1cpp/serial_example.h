#ifndef SERIAL_EXAMPLE_H
#define SERIAL_EXAMPLE_H
extern "C" {
#include "er_ti_f28069m_drv8305/er_buffer.h"
#include "er_ti_f28069m_drv8305/er_command.h"
#include "er_ti_f28069m_drv8305/er_msg.h"
#include "er_ti_f28069m_drv8305/er_registers.h"
}
#include <fcntl.h>
#include <pthread.h>
#include <ros/ros.h>
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
    const float &serial_read();
    std::string get_pos_cmd(uint16_t source_id, uint16_t node_id);
    std::string set_pos_cmd(uint16_t source_id, uint16_t node_id, float position);
    std::string ping_cmd(uint16_t source_id, uint16_t node_id);
    void ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port);
    std::string hexStr(unsigned char *data, int len);
    void write_callback(const std_msgs::String::ConstPtr &msg);

protected:
    int serial_port_fd;

    int no_read_bytes = 0;
    std::vector<uint8_t> serial_read_buffer;
    std::vector<uint16_t> ti_read_buffer;

    int no_written_bytes = 0;

    char *ping_byte_array, *get_pos_byte_array, *set_pos_byte_array; //= "";
    std::string ping_byte_str, get_pos_byte_str, set_pos_byte_str;
    uint16_t ping_cmd_len, get_pos_cmd_len, set_pos_cmd_len;
    float current_position;
    float desired_pose = 0.0;

    ros::NodeHandle nh_;
    ros::Subscriber write_sub;
    ros::Publisher read_pub;
    ros::Publisher write_pub_str;

    volatile unsigned char _RX_SCI_buf[1024]; // buffer RX
    volatile uint16_t _RX_SCI_ptr = 0;
    volatile uint16_t __flag_buffer_rx_is_changed = 0;
    volatile uint16_t _node_id = 1;
    volatile ER_Msg gMsgCommand = ER_Msg_INIT;
    volatile ER_Msg gMsgResponse = ER_Msg_INIT;
};
#endif
