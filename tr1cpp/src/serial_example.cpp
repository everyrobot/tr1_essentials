//#include "tr1cpp/serial_example.h"
//
//Serial::Serial(ros::NodeHandle &nh, uint16_t source_id, uint16_t node_id, std::string serial_port)
//    : nh_(nh)
//{
//    serial_read_buffer.resize(32);
//    ti_read_buffer.resize(13);
//    //ros_loop(source_id, node_id, serial_port);
//    //write_sub = nh_.subscribe("write_", 1000, &Serial::write_callback, this);
//    //read_pub = nh_.advertise<std_msgs::String>("read", 1000);
//    //write_pub_str = nh_.advertise<std_msgs::String>("write_", 1000);
//}
//
//Serial::~Serial() {}
//
//bool Serial::serial_init(std::string serial_port)
//{
//    serial_port_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//    struct termios tty;
//    struct termios tty_old;
//    memset(&tty, 0, sizeof tty);
//
//    /* Error Handling */
//    if (tcgetattr(serial_port_fd, &tty) != 0) {
//        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
//        return false;
//    }
//
//    /* Save old tty parameters */
//    tty_old = tty;
//
//    /* Set Baud Rate */
//    cfsetospeed(&tty, (speed_t) B115200);
//    cfsetispeed(&tty, (speed_t) B115200);
//
//    /* Setting other Port Stuff */
//    tty.c_cflag &= ~PARENB; // Make 8n1
//    tty.c_cflag &= ~CSTOPB;
//    tty.c_cflag &= ~CSIZE;
//    tty.c_cflag |= CS8;
//
//    tty.c_cflag &= ~CRTSCTS;       // no flow control
//    tty.c_cc[VMIN] = 1;            // read doesn't block
//    tty.c_cc[VTIME] = 0;           //5         // 0.5 seconds read timeout
//    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
//
//    /* Make raw */
//    cfmakeraw(&tty);
//
//    /* Flush Port, then applies attributes */
//    tcflush(serial_port_fd, TCIFLUSH);
//    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
//        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
//        return false;
//    }
//    return true;
//}
//
//int Serial ::serial_write(const std::string &data)
//{
//    no_written_bytes = write(serial_port_fd, data.c_str(), data.size());
//    return no_written_bytes;
//}
//
//const float &Serial ::serial_read()
//{
//    do {
//        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
//        for (int i = 0; i < no_read_bytes; i += 1) {
//            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
//            buffer__add_to_buffer(_RX_SCI_buf,
//                                  &_RX_SCI_ptr,
//                                  &__flag_buffer_rx_is_changed,
//                                  ti_read_buffer[i]);
//        }
//    } while (!buffer__analyse_buffer(_RX_SCI_buf,
//                                     &_RX_SCI_ptr,
//                                     &__flag_buffer_rx_is_changed,
//                                     &_node_id,
//                                     &gMsgResponse));
//
//    if (command__check_msg_flags(&gMsgResponse) == 0
//        && command__check_response_msg(&gMsgResponse) == 0) {
//        if (gMsgResponse.body_size != 0) {
//            current_position = msg__get_float_from_body(&gMsgResponse, 0);
//            ROS_INFO("current position is :%f ", current_position);
//            return current_position;
//        } else {
//            ROS_INFO("Position is set ");
//        }
//    }
//}
//
//std::string Serial::get_pos_cmd(uint16_t source_id, uint16_t node_id)
//{
//    ER_Msg get_pos_cmd = command__create_msg_get_position(source_id, node_id);
//    get_pos_cmd_len = msg__get_length(&get_pos_cmd);
//    get_pos_byte_array = msg__to_byte_array(&get_pos_cmd);
//    ////std::string str_hex_set_pos = hexStr((unsigned char *) get_pos_byte_array, get_pos_cmd_len);
//    get_pos_byte_str.assign(get_pos_byte_array, get_pos_cmd_len);
//    return get_pos_byte_str;
//}
//std::string Serial::set_pos_cmd(uint16_t source_id, uint16_t node_id, float position)
//{
//    ER_Msg set_pos_cmd = command__create_msg_set_position(source_id, node_id, position);
//    set_pos_cmd_len = msg__get_length(&set_pos_cmd);
//    set_pos_byte_array = msg__to_byte_array(&set_pos_cmd);
//    //std::string str_hex_set_pos = hexStr((unsigned char *) set_pos_byte_array, set_pos_cmd_len);
//    set_pos_byte_str.assign(set_pos_byte_array, set_pos_cmd_len);
//    return set_pos_byte_str;
//}
//std::string Serial::ping_cmd(uint16_t source_id, uint16_t node_id)
//{
//    // Creating TI Command Message
//    ER_Msg ping_cmd = command__create_msg_ping(source_id, node_id);
//    // Getting Lengths of TI Command Message
//    ping_cmd_len = msg__get_length(&ping_cmd);
//    // Creating byte arrays from TI Command Message
//    ping_byte_array = msg__to_byte_array(&ping_cmd);
//    // (Debugging) Creating hex strings from byte array
//    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
//    // (Display) Show hex strings of TI Commands Message
//    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
//    // Creating data strings from byte arrays for ROS compatibility
//    ping_byte_str.assign(ping_byte_array, ping_cmd_len);
//    // Passing data strings of TI Command Message into ROS Msg
//    //str_ros_ping.data = str_byte_ping;
//    return ping_byte_str;
//}
//void Serial::ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port)
//{
//    ros::Rate loop_rate(100);
//    if (serial_init(serial_port)) {
//        while (ros::ok()) {
//            serial_write(get_pos_cmd(source_id, node_id));
//            serial_read();
//            serial_write(set_pos_cmd(source_id, node_id, desired_pose));
//            serial_read();
//            desired_pose += 0.05;
//            if (desired_pose > 120.0)
//                desired_pose = 0.0;
//            loop_rate.sleep();
//        }
//    }
//}
//std::string Serial::hexStr(unsigned char *data, int len)
//{
//    constexpr char hexmap[]
//        = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
//    std::string s(len * 2, ' ');
//    for (int i = 0; i < len; ++i) {
//        s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
//        s[2 * i + 1] = hexmap[data[i] & 0x0F];
//    }
//    return s;
//}
//
//void Serial::write_callback(const std_msgs::String::ConstPtr &msg) {}
#include "tr1cpp/serial_example.h"

Serial::Serial(ros::NodeHandle &nh, uint16_t source_id, uint16_t node_id, std::string serial_port)
    : nh_(nh)
{
    serial_read_buffer.resize(32);
    ti_read_buffer.resize(13);
    write_sub = nh_.subscribe("joint_set_pose", 1000, &Serial::set_pose_callback, this);
    read_pub = nh_.advertise<std_msgs::Float64>("joint_get_pose", 1000);
    pcb1_sensors_voltage_pub = nh_.advertise<std_msgs::Float64MultiArray>("sensors_voltage", 1000);
    //ros_loop(source_id, node_id, serial_port);

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

void Serial::check_response_type(volatile ER_Msg *_msg,
                                 std::vector<uint16_t> *serial_read_tactile,
                                 float *current_pose)
{
    //ROS_INFO("check_response_type:%d ", (*_msg).register_id);
    switch ((*_msg).register_id) {
    case ER_REG_PING:              // PING message
    case ER_REG_SET_POSITION_SP: { // Set position setpoint
        if ((*_msg).body_size == 0)
            ROS_INFO("Position is set ");
        break;
    }

    case ER_REG_GET_POSITION: { // Set position setpoint
        if ((*_msg).body_size == 4) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            *current_pose = current_position;
            ROS_INFO("current position is :%f ", current_position);
            get_pose_msg.data = current_position;
            read_pub.publish(get_pose_msg);
        }

        break;
    }
    case ER_TACTILE_PCB1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        pcb1_voltages_msg.data.clear();
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i <= (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2) {
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
                serial_read_tactile->at(j++) = tactile_serial_read[j++];
            }
            for (int i = 0; i < (int) tactile_serial_read.size(); i++) {
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
                pcb1_voltages_msg.data.push_back(tactile_serial_read.at(i));
            }
            pcb1_sensors_voltage_pub.publish(pcb1_voltages_msg);
        }
        break;
    }

    case ER_TACTILE_PCB2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }

    case ER_TACTILE_FINGER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB3_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB4_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_FINGER2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_GRIPPER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0;
                 i < (NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2);
                 i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    default: {}
    }
}

void Serial::check_response_type(volatile ER_Msg *_msg, float *current_pose)
{
    //ROS_INFO("check_response_type:%d ", (*_msg).register_id);
    switch ((*_msg).register_id) {
    case ER_REG_PING:              // PING message
    case ER_REG_SET_POSITION_SP: { // Set position setpoint
        if ((*_msg).body_size == 0)
            ROS_INFO("Position is set ");
        break;
    }

    case ER_REG_GET_POSITION: { // Set position setpoint
        if ((*_msg).body_size == 4) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            *current_pose = current_position;
            ROS_INFO("current position is :%f ", current_position);
            get_pose_msg.data = current_position;
            read_pub.publish(get_pose_msg);
        }

        break;
    }
    case ER_TACTILE_PCB1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        pcb1_voltages_msg.data.clear();
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i <= (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2) {
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
                //serial_read_tactile->at(j++) = tactile_serial_read[j++];
            }
            for (int i = 0; i < (int) tactile_serial_read.size(); i++) {
                //ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
                pcb1_voltages_msg.data.push_back(tactile_serial_read.at(i));
            }
            pcb1_sensors_voltage_pub.publish(pcb1_voltages_msg);
        }
        break;
    }

    case ER_TACTILE_PCB2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }

    case ER_TACTILE_FINGER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB3_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB4_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_FINGER2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_GRIPPER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0;
                 i < (NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2);
                 i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    default: {}
    }
}

void Serial::check_response_type(volatile ER_Msg *_msg, std::vector<uint16_t> *serial_read_tactile)
{
    //ROS_INFO("check_response_type:%d ", (*_msg).register_id);
    switch ((*_msg).register_id) {
    case ER_REG_PING:              // PING message
    case ER_REG_SET_POSITION_SP: { // Set position setpoint
        if ((*_msg).body_size == 0)
            ROS_INFO("Position is set ");
        break;
    }

    case ER_REG_GET_POSITION: { // Set position setpoint
        if ((*_msg).body_size == 4) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            //*current_pose = current_position;
            //ROS_INFO("current position is :%f ", current_position);
            get_pose_msg.data = current_position;
            read_pub.publish(get_pose_msg);
        }

        break;
    }
    case ER_TACTILE_PCB1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        pcb1_voltages_msg.data.clear();
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i <= (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2) {
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
                //serial_read_tactile->at(j++) = tactile_serial_read[j++];
            }
            for (int i = 0; i < (int) tactile_serial_read.size(); i++) {
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
                serial_read_tactile->at(i) = tactile_serial_read.at(i);
                pcb1_voltages_msg.data.push_back(tactile_serial_read.at(i));
            }
            pcb1_sensors_voltage_pub.publish(pcb1_voltages_msg);
        }
        break;
    }

    case ER_TACTILE_PCB2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }

    case ER_TACTILE_FINGER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB3_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB4_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_FINGER2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_GRIPPER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0;
                 i < (NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2);
                 i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    default: {}
    }
}

void Serial::check_response_type(volatile ER_Msg *_msg)
{
    //ROS_INFO("check_response_type:%d ", (*_msg).register_id);
    switch ((*_msg).register_id) {
    case ER_REG_PING:              // PING message
    case ER_REG_SET_POSITION_SP: { // Set position setpoint
        if ((*_msg).body_size == 0)
            ROS_INFO("Position is set ");
        break;
    }

    case ER_REG_GET_POSITION: { // Set position setpoint
        if ((*_msg).body_size == 4) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            //*current_pose = current_position;
            ROS_INFO("current position is :%f ", current_position);
            get_pose_msg.data = current_position;
            read_pub.publish(get_pose_msg);
        }

        break;
    }
    case ER_TACTILE_PCB1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        pcb1_voltages_msg.data.clear();
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i <= (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2) {
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
                //serial_read_tactile->at(j++) = tactile_serial_read[j++];
            }
            for (int i = 0; i < (int) tactile_serial_read.size(); i++) {
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
                pcb1_voltages_msg.data.push_back(tactile_serial_read.at(i));
            }
            pcb1_sensors_voltage_pub.publish(pcb1_voltages_msg);
        }
        break;
    }

    case ER_TACTILE_PCB2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }

    case ER_TACTILE_FINGER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB3_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_PCB4_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_FINGER2_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0; i < (NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2); i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    case ER_TACTILE_GRIPPER1_GET_MEDIAN: { // Set position setpoint
        tactile_serial_read.resize(NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS);
        int j = 0;
        if ((*_msg).body_size == NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t)) {
            for (int i = 0;
                 i < (NO_OF_FINGERS * NO_OF_PCBS * NO_OF_CHANNELS * sizeof(uint16_t) - 2);
                 i = i + 2)
                tactile_serial_read[j++] = (msg__get_uint16_from_body(&gMsgResponse, i));
            for (int i = 0; i < (int) tactile_serial_read.size(); i++)
                ROS_INFO("tactile_serial_read(%d) is :%d", i, tactile_serial_read.at(i));
        }

        break;
    }
    default: {}
    }
}

void Serial ::serial_read(std::vector<uint16_t> *tactile_serial_read, float *current_pose)
{
    do {
        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
        //ROS_INFO(" serial_read: no_read_bytes is :%d", no_read_bytes);
        for (int i = 0; i < no_read_bytes; i += 1) {
            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
            //ROS_INFO(" serial_read: read_byteis :%d", ti_read_buffer[i]);
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
        check_response_type(&gMsgResponse, tactile_serial_read, current_pose);
    }
}

void Serial ::serial_read(float *current_pose)
{
    do {
        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
        //ROS_INFO(" serial_read: no_read_bytes is :%d", no_read_bytes);
        for (int i = 0; i < no_read_bytes; i += 1) {
            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
            //ROS_INFO(" serial_read: read_byteis :%d", ti_read_buffer[i]);
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
        check_response_type(&gMsgResponse, current_pose);
    }
}

void Serial ::serial_read(std::vector<uint16_t> *tactile_serial_read)
{
    do {
        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
        //ROS_INFO(" serial_read: no_read_bytes is :%d", no_read_bytes);
        for (int i = 0; i < no_read_bytes; i += 1) {
            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
            //ROS_INFO(" serial_read: read_byteis :%d", ti_read_buffer[i]);
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
        check_response_type(&gMsgResponse, tactile_serial_read);
    }
}


void Serial ::serial_read()
{
    do {
        no_read_bytes = read(serial_port_fd, &serial_read_buffer[0], 32);
        //ROS_INFO(" serial_read: no_read_bytes is :%d", no_read_bytes);
        for (int i = 0; i < no_read_bytes; i += 1) {
            ti_read_buffer[i] = static_cast<uint16_t>(serial_read_buffer[i]);
            //ROS_INFO(" serial_read: read_byteis :%d", ti_read_buffer[i]);
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
        check_response_type(&gMsgResponse);
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

std::string Serial::tactile_pcb1_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_pcb1_get_median_cmd = command__create_msg_tactile_pcb1_get_median(source_id,
                                                                                     node_id);
    // Getting Lengths of TI Command Message
    pcb1_cmd_len = msg__get_length(&tactile_pcb1_get_median_cmd);
    // Creating byte arrays from TI Command Message
    pcb1_byte_array = msg__to_byte_array(&tactile_pcb1_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_pcb1 = hexStr((unsigned char *) pcb1_byte_array, pcb1_cmd_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_pcb1);
    // Creating data strings from byte arrays for ROS compatibility
    pcb1_byte_str.assign(pcb1_byte_array, pcb1_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return pcb1_byte_str;
}

std::string Serial::tactile_pcb2_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_pcb2_get_median = command__create_msg_tactile_pcb2_get_median(source_id, node_id);
    // Getting Lengths of TI Command Message
    pcb2_cmd_len = msg__get_length(&tactile_pcb2_get_median);
    // Creating byte arrays from TI Command Message
    pcb2_byte_array = msg__to_byte_array(&tactile_pcb2_get_median);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_pcb2 = hexStr((unsigned char *) pcb2_byte_array, pcb2_cmd_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_pcb2);
    // Creating data strings from byte arrays for ROS compatibility
    pcb2_byte_str.assign(pcb2_byte_array, pcb2_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return pcb2_byte_str;
}

std::string Serial::tactile_finger1_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_finger1_get_median_cmd = command__create_msg_tactile_finger1_get_median(source_id,
                                                                                           node_id);
    // Getting Lengths of TI Command Message
    fnger1_cmd_len = msg__get_length(&tactile_finger1_get_median_cmd);
    // Creating byte arrays from TI Command Message
    finger1_byte_array = msg__to_byte_array(&tactile_finger1_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    finger1_byte_str.assign(finger1_byte_array, fnger1_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return finger1_byte_str;
}

std::string Serial::tactile_pcb3_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_pcb3_get_median_cmd = command__create_msg_tactile_pcb3_get_median(source_id,
                                                                                     node_id);
    // Getting Lengths of TI Command Message
    pcb3_cmd_len = msg__get_length(&tactile_pcb3_get_median_cmd);
    // Creating byte arrays from TI Command Message
    pcb3_byte_array = msg__to_byte_array(&tactile_pcb3_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    pcb3_byte_str.assign(pcb3_byte_array, pcb3_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return pcb3_byte_str;
}

std::string Serial::tactile_pcb4_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_pcb4_get_median_cmd = command__create_msg_tactile_pcb4_get_median(source_id,
                                                                                     node_id);
    // Getting Lengths of TI Command Message
    pcb4_cmd_len = msg__get_length(&tactile_pcb4_get_median_cmd);
    // Creating byte arrays from TI Command Message
    pcb4_byte_array = msg__to_byte_array(&tactile_pcb4_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    pcb4_byte_str.assign(pcb4_byte_array, pcb4_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return pcb4_byte_str;
}

std::string Serial::tactile_finger2_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_finger2_get_median_cmd = command__create_msg_tactile_finger2_get_median(source_id,
                                                                                           node_id);
    // Getting Lengths of TI Command Message
    fnger2_cmd_len = msg__get_length(&tactile_finger2_get_median_cmd);
    // Creating byte arrays from TI Command Message
    finger2_byte_array = msg__to_byte_array(&tactile_finger2_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    finger2_byte_str.assign(finger2_byte_array, fnger2_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return finger2_byte_str;
}

std::string Serial::tactile_gripper1_get_median_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg tactile_gripper1_get_median_cmd
        = command__create_msg_tactile_gripper1_get_median(source_id, node_id);
    // Getting Lengths of TI Command Message
    gripper1_cmd_len = msg__get_length(&tactile_gripper1_get_median_cmd);
    // Creating byte arrays from TI Command Message
    gripper1_byte_array = msg__to_byte_array(&tactile_gripper1_get_median_cmd);
    // (Debugging) Creating hex strings from byte array
    //std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    griper1_byte_str.assign(gripper1_byte_array, gripper1_cmd_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return griper1_byte_str;
}

void Serial::ros_loop(uint16_t source_id, uint16_t node_id, std::string serial_port)
{
    //tactile_force_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_pcb1_median", 1000);

    //array1_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array1_median", 1000);
    //array2_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array2_median", 1000);
    //array3_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array3_median", 1000);
    //array4_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array4_median", 1000);
    //array5_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array5_median", 1000);
    //array6_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array6_median", 1000);
    //array7_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array7_median", 1000);
    //array8_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array8_median", 1000);
    //array9_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_array9_median", 1000);

    //geometry_msgs::WrenchStamped pcb1_msg;

    //geometry_msgs::WrenchStamped array1_msg;
    //geometry_msgs::WrenchStamped array2_msg;
    //geometry_msgs::WrenchStamped array3_msg;
    //geometry_msgs::WrenchStamped array4_msg;
    //geometry_msgs::WrenchStamped array5_msg;
    //geometry_msgs::WrenchStamped array6_msg;
    //geometry_msgs::WrenchStamped array7_msg;
    //geometry_msgs::WrenchStamped array8_msg;
    //geometry_msgs::WrenchStamped array9_msg;

    //pcb1_msg.header.frame_id = "/map";

    //array1_msg.header.frame_id = "/map";
    //array2_msg.header.frame_id = "/map";
    //array3_msg.header.frame_id = "/map";
    //array4_msg.header.frame_id = "/map";
    //array5_msg.header.frame_id = "/map";
    //array6_msg.header.frame_id = "/map";
    //array7_msg.header.frame_id = "/map";
    //array8_msg.header.frame_id = "/map";
    //array9_msg.header.frame_id = "/map";

    //uint8_t pcb_visual_downscale_factor = 35;
    //uint8_t array_downscale_factor = 5;
    ros::Rate loop_rate(100);
    if (serial_init(serial_port)) {
        while (ros::ok()) {
            serial_write(get_pos_cmd(source_id, node_id));
            serial_read();

            serial_write(tactile_pcb1_get_median_cmd(source_id, node_id));
            serial_read();

            //pcb1_msg.header.stamp = ros::Time::now();

            //array1_msg.header.stamp = ros::Time::now();
            //array2_msg.header.stamp = ros::Time::now();
            //array3_msg.header.stamp = ros::Time::now();
            //array4_msg.header.stamp = ros::Time::now();
            //array5_msg.header.stamp = ros::Time::now();
            //array6_msg.header.stamp = ros::Time::now();
            //array7_msg.header.stamp = ros::Time::now();
            //array8_msg.header.stamp = ros::Time::now();
            //array9_msg.header.stamp = ros::Time::now();

            //pcb1_msg.wrench.force.z = 0;

            //array1_msg.wrench.force.z = 0;
            //array2_msg.wrench.force.z = 0;
            //array3_msg.wrench.force.z = 0;
            //array4_msg.wrench.force.z = 0;
            //array5_msg.wrench.force.z = 0;
            //array6_msg.wrench.force.z = 0;
            //array7_msg.wrench.force.z = 0;
            //array8_msg.wrench.force.z = 0;
            //array9_msg.wrench.force.z = 0;

            //for (int i = 0; i < NO_OF_CHANNELS; i++)
            // pcb1_msg.wrench.force.z += tactile_serial_read[i] / pcb_visual_downscale_factor;

            //array1_msg.wrench.force.z += tactile_serial_read[0] / array_downscale_factor;
            //array2_msg.wrench.force.z += tactile_serial_read[1] / array_downscale_factor;
            //array3_msg.wrench.force.z += tactile_serial_read[2] / array_downscale_factor;
            //array4_msg.wrench.force.z += tactile_serial_read[3] / array_downscale_factor;
            //array5_msg.wrench.force.z += tactile_serial_read[4] / array_downscale_factor;
            //array6_msg.wrench.force.z += tactile_serial_read[5] / array_downscale_factor;
            //array7_msg.wrench.force.z += tactile_serial_read[6] / array_downscale_factor;
            //array8_msg.wrench.force.z += tactile_serial_read[7] / array_downscale_factor;
            //array9_msg.wrench.force.z += tactile_serial_read[8] / array_downscale_factor;

            //tactile_force_pub.publish(pcb1_msg);

            //array1_pub.publish(array1_msg);
            //array2_pub.publish(array2_msg);
            //array3_pub.publish(array3_msg);
            //array4_pub.publish(array4_msg);
            //array5_pub.publish(array5_msg);
            //array6_pub.publish(array6_msg);
            //array7_pub.publish(array7_msg);
            //array8_pub.publish(array8_msg);
            //array9_pub.publish(array9_msg);

            //serial_write(tactile_pcb2_get_median_cmd(source_id, node_id));
            //serial_read();
            //serial_write(tactile_pcb3_get_median_cmd(source_id, node_id));
            //serial_read();
            //serial_write(tactile_pcb4_get_median_cmd(source_id, node_id));
            //serial_read();
            //serial_write(tactile_finger1_get_median_cmd(source_id, node_id));
            //serial_read();
            //serial_write(tactile_finger2_get_median_cmd(source_id, node_id));
            //serial_read();
            //serial_write(tactile_gripper1_get_median_cmd(source_id, node_id));
            //serial_read();
            //
            //serial_write(set_pos_cmd(1, 100, desired_pose));
            //serial_read();
            //desired_pose += 0.05;

            //if (desired_pose > 120.0)
            //desired_pose = 0.0;
            ros::Duration(0.005).sleep();

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

void Serial::set_pose_callback(const std_msgs::Float64::ConstPtr &msg)
{
    //desired_pose = msg->data;
    //serial_write(set_pos_cmd(1, 100, desired_pose));
    //serial_read();
    //desired_pose += 0.05;
}

//void Serial::get_pose_callback(const std_msgs::Float64::ConstPtr &msg)
//{
//    double request = msg->data;
//    serial_write(get_pos_cmd(1, 100));
//    serial_read();
//    //desired_pose += 0.05;
//}
