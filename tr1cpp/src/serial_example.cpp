#include "tr1cpp/serial_example.h"

Serial::Serial(ros::NodeHandle &nh, uint16_t source_id, uint16_t node_id, std::string serial_port)
    : nh_(nh)
{
    serial_read_buffer.resize(32);
    ti_read_buffer.resize(13);
    //ros_loop(source_id, node_id, serial_port);
    //write_sub = nh_.subscribe("write_", 1000, &Serial::write_callback, this);
    //read_pub = nh_.advertise<std_msgs::String>("read", 1000);
    //write_pub_str = nh_.advertise<std_msgs::String>("write_", 1000);
}

Serial::~Serial() {}

bool Serial::serial_init(std::string serial_port)
{
    serial_port_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(serial_port_fd, &tty) != 0) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t) B115200);
    cfsetispeed(&tty, (speed_t) B115200);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN] = 1;            // read doesn't block
    tty.c_cc[VTIME] = 0;           //5         // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush(serial_port_fd, TCIFLUSH);
    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return false;
    }
    return true;
}

int Serial ::serial_write(const std::string &data)
{
    no_written_bytes = write(serial_port_fd, data.c_str(), data.size());
    return no_written_bytes;
}

const float &Serial ::serial_read()
{
    do {
        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
        for (int i = 0; i < no_read_bytes; i += 1) {
            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
            buffer__add_to_buffer(_RX_SCI_buf,
                                  &_RX_SCI_ptr,
                                  &__flag_buffer_rx_is_changed,
                                  ti_read_buffer[i]);
        }
    } while (!buffer__analyse_buffer(_RX_SCI_buf,
                                     &_RX_SCI_ptr,
                                     &__flag_buffer_rx_is_changed,
                                     &_node_id,
                                     &gMsgResponse));

    if (command__check_msg_flags(&gMsgResponse) == 0
        && command__check_response_msg(&gMsgResponse) == 0) {
        if (gMsgResponse.body_size != 0) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            ROS_INFO("current position is :%f ", current_position);
            return current_position;
        } else {
            ROS_INFO("Position is set ");
        }
    }
}

std::string Serial::get_pos_cmd(uint16_t source_id, uint16_t node_id)
{
    ER_Msg get_pos_cmd = command__create_msg_get_position(source_id, node_id);
    get_pos_cmd_len = msg__get_length(&get_pos_cmd);
    get_pos_byte_array = msg__to_byte_array(&get_pos_cmd);
    ////std::string str_hex_set_pos = hexStr((unsigned char *) get_pos_byte_array, get_pos_cmd_len);
    get_pos_byte_str.assign(get_pos_byte_array, get_pos_cmd_len);
    return get_pos_byte_str;
}
std::string Serial::set_pos_cmd(uint16_t source_id, uint16_t node_id, float position)
{
    ER_Msg set_pos_cmd = command__create_msg_set_position(source_id, node_id, position);
    set_pos_cmd_len = msg__get_length(&set_pos_cmd);
    set_pos_byte_array = msg__to_byte_array(&set_pos_cmd);
    //std::string str_hex_set_pos = hexStr((unsigned char *) set_pos_byte_array, set_pos_cmd_len);
    set_pos_byte_str.assign(set_pos_byte_array, set_pos_cmd_len);
    return set_pos_byte_str;
}
std::string Serial::ping_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg ping_cmd = command__create_msg_ping(source_id, node_id);
    // Getting Lengths of TI Command Message
    ping_cmd_len = msg__get_length(&ping_cmd);
    // Creating byte arrays from TI Command Message
    ping_byte_array = msg__to_byte_array(&ping_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    ping_byte_str.assign(ping_byte_array, ping_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return ping_byte_str;
}
void Serial::ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port)
{
    ros::Rate loop_rate(100);
    if (serial_init(serial_port)) {
        while (ros::ok()) {
            serial_write(get_pos_cmd(source_id, node_id));
            serial_read();
            serial_write(set_pos_cmd(source_id, node_id, desired_pose));
            serial_read();
            desired_pose += 0.05;
            if (desired_pose > 120.0)
                desired_pose = 0.0;
            loop_rate.sleep();
        }
    }
}
std::string Serial::hexStr(unsigned char *data, int len)
{
    constexpr char hexmap[]
        = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    std::string s(len * 2, ' ');
    for (int i = 0; i < len; ++i) {
        s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
        s[2 * i + 1] = hexmap[data[i] & 0x0F];
    }
    return s;
}

void Serial::write_callback(const std_msgs::String::ConstPtr &msg) {}
