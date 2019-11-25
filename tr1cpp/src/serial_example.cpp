#include <tr1cpp/serial_example.h>

Serial::Serial(ros::NodeHandle &nh,
               uint16_t source_id,
               uint16_t node_id,
               float &current_pos,
               float desired_pos)
    : nh_(nh)
{
    //write_sub = nh_.subscribe("write_", 1000, &Serial::write_callback, this);
    //read_pub = nh_.advertise<std_msgs::String>("read", 1000);
    //write_pub_str = nh_.advertise<std_msgs::String>("write_", 1000);
    //ros_loop(source_id, node_id, current_pos, desired_pos);
}

Serial::~Serial() {}
bool Serial::init()
{
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10); //1000 //10
        ser.setTimeout(to);
        ser.open();
        //ser.flush();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return false;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return false;
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

void Serial::write_callback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Writing to serial port:  size %d", msg->data.size());
    //ser.write(msg->data);
    //std::cout << " write elapsed_time is : " << (ros::Time::now() - start_time) * 1000 << " msec"
    //        << std::endl;
}

std::string Serial::ping_cmd(uint16_t source_id, uint16_t node_id)
{
    // Creating TI Command Message
    ER_Msg msg_ping = command__create_msg_ping(source_id, node_id);
    // Getting Lengths of TI Command Message
    msg_ping_len = msg__get_length(&msg_ping);
    // Creating byte arrays from TI Command Message
    byte_array_ping = msg__to_byte_array(&msg_ping);
    // (Debugging) Creating hex strings from byte array
    std::string str_hex_ping = hexStr((unsigned char *) byte_array_ping, msg_ping_len);
    // (Display) Show hex strings of TI Commands Message
    //ROS_INFO_STREAM("hexstring write " << str_hex_ping);
    // Creating data strings from byte arrays for ROS compatibility
    std::string str_byte_ping(byte_array_ping, msg_ping_len);
    // Passing data strings of TI Command Message into ROS Msg
    //str_ros_ping.data = str_byte_ping;
    return str_byte_ping;
}
std::string Serial::get_pos_cmd(uint16_t source_id, uint16_t node_id)
{
    ER_Msg cmd_get_pos = command__create_msg_get_position(source_id, node_id);
    cmd_get_pos_len = msg__get_length(&cmd_get_pos);
    byte_array_get_pos = msg__to_byte_array(&cmd_get_pos);
    std::string str_hex_get_pos = hexStr((unsigned char *) byte_array_get_pos, cmd_get_pos_len);
    //ROS_INFO_STREAM("hexstring write " << str_hex_get_pos);
    //std::ios_base::sync_with_stdio(false);
    // std::cout << "  hexstring write " << str_hex_get_pos << '\n';
    std::string str_byte_get_pos(byte_array_get_pos, cmd_get_pos_len);
    //str_ros_get_pos.data = str_byte_get_pos;
    return str_byte_get_pos;
}
std::string Serial::set_pos_cmd(uint16_t source_id, uint16_t node_id, float position)
{
    ER_Msg cmd_set_pos = command__create_msg_set_position(source_id, node_id, position);
    cmd_set_pos_len = msg__get_length(&cmd_set_pos);
    byte_array_set_pos = msg__to_byte_array(&cmd_set_pos);
    std::string str_hex_set_pos = hexStr((unsigned char *) byte_array_set_pos, cmd_set_pos_len);
    //std::cout << "  hexstring write " << str_hex_set_pos << '\n';
    //std::cout << "  hexstring write size " << cmd_set_pos_len << '\n';
    //ROS_INFO_STREAM("hexstring write " << str_hex_set_pos);
    std::string str_byte_set_pos(byte_array_set_pos, cmd_set_pos_len);
    //str_ros_set_pos.data = str_byte_set_pos;
    return str_byte_set_pos;
}

void Serial::serial_read()
{
    // Reading from Serial in string
    data = ser.read(ser.available());
    // Creating hex string from serial string
    serial_read_hexstr = hexStr((unsigned char *) data.c_str(), data.length());
    //  (Display) Show hex strings of TI Response Message
    //ROS_INFO_STREAM("hexstring read: " << serial_read_hexstr);
    //std::ios_base::sync_with_stdio(false);
    //std::cout << "hexstring read: " << serial_read_hexstr << '\n';
    //std::cout << "hexstring read size: " << data.length() << '\n';
    //ROS_INFO("Reading from serial port: size %d", data.length());
    // Converting string to char array(uint8_t array)
    serial_read_char_array = (unsigned char *) data.c_str();
    uint16_t serial_read[data.length()];

    for (int i = 0; i < data.size(); i += 1) {
        serial_read[i] = 0x0000;
        // Converting char array(uint8_t array) to uint16_t array)
        serial_read[i] |= serial_read_char_array[i];
        //ROS_INFO("byte%d :%d ", i, serial_read[i]);
        // Adding byte by byte to Buffer
        buffer__add_to_buffer(_RX_SCI_buf,
                              &_RX_SCI_ptr,
                              &__flag_buffer_rx_is_changed,
                              serial_read[i]);
    }
}
//void Serial::serial_write(std::string serial_write_ros)
//{
//    // Writing to Serial by ROS publish
//    //write_pub_str.publish(serial_write_ros);
//    //start_time = ros::Time::now();
//    //start = clock();
//    //ser.write(serial_write_ros);
//}

bool Serial::get_pos_resp(float &pos)
{
    // Analyzing the Buffer to get the response and data
    if (buffer__analyse_buffer(_RX_SCI_buf,
                               &_RX_SCI_ptr,
                               &__flag_buffer_rx_is_changed,
                               &_node_id,
                               &gMsgResponse)) {
        //uint16_t cmd_get_pos_len_ = msg__get_length(&gMsgResponse);
        //char *byte_array_get_pos_ = msg__to_byte_array(&gMsgResponse);
        //std::string str_hex_get_pos_ = hexStr((unsigned char *) byte_array_get_pos_,
        //                                      cmd_get_pos_len_);
        //ROS_INFO_STREAM("hexstring READ " << str_hex_get_pos_);

        if (command__check_msg_flags(&gMsgResponse) == 0
            && command__check_response_msg(&gMsgResponse) == 0) {
            current_position = msg__get_float_from_body(&gMsgResponse, 0);
            //std::cout << "  current position " << current_position << '\n';
            //ROS_INFO("current position is :%f ", current_position);
            pos = current_position;
            return true;

        } else {
            // BLA BLA BLA - ACTION ON ERRORS
            return false;
        }
    }
}

bool Serial::set_pos_resp()
{
    if (buffer__analyse_buffer(_RX_SCI_buf,
                               &_RX_SCI_ptr,
                               &__flag_buffer_rx_is_changed,
                               &_node_id,
                               &gMsgResponse)) {
        if (command__check_msg_flags(&gMsgResponse) == 0
            && command__check_response_msg(&gMsgResponse) == 0) {
            ROS_INFO("position is set ");
            return true;
        } else {
            return false;
            //BLA BLA BLA - ACTION ON ERRORS
        }
    }
}

void Serial::get_pos_read(float &pos, double elapsed_time)
{
    // ROS_INFO("Serial::get_pos_read starts");

    //std::cout << " current time is : " << begin_time << std::endl;
    //ros::Time begin_time;
    while (!get_pos_resp(pos)) {
        //while (1) {
        //begin_time = ros::Time::now();
        //ROS_INFO("read serial starts");
        //std::cout << ser.available() << '\n';
        if (ser.available()) { //
            //ROS_INFO("serial reading");
            serial_read();
            //serial.set_pos_resp();
        }
        // if ((ros::Time::now() - begin_time) > elapsed_time) {
        //     ios_base::sync_with_stdio(false);
        //     std::cout << " Long read elapsed_time is : " << (ros::Time::now() - begin_time) * 1000
        //               << " msec" << '\n';
        //     return false;
        // }
    }
    // ios_base::sync_with_stdio(false);
    // std::cout << " Short read elapsed_time is : " << (ros::Time::now() - begin_time) * 1000
    //           << " msec" << '\n';
    // return true;
    // ROS_INFO("Serial::get_pos_read ends");
}

void Serial::set_pos_read(double elapsed_time)
{
    //ROS_INFO("Serial::set_pos_read starts");

    //std::cout << " current time is : " << begin_time << std::endl;
    //ros::Time begin_time;
    while (!set_pos_resp()) {
        //while (1) {
        //begin_time = ros::Time::now();
        //ROS_INFO("read serial starts");
        //std::cout << ser.available() << '\n';
        if (ser.available()) { //
            //ROS_INFO("serial reading");
            serial_read();
            //serial.set_pos_resp();
        }
        // if ((ros::Time::now() - begin_time) > elapsed_time) {
        //     ios_base::sync_with_stdio(false);
        //     std::cout << " Long read elapsed_time is : " << (ros::Time::now() - begin_time) * 1000
        //               << " msec" << '\n';
        //     return false;
        // }
    }
    // ios_base::sync_with_stdio(false);
    // std::cout << " Short read elapsed_time is : " << (ros::Time::now() - begin_time) * 1000
    //           << " msec" << '\n';
    // return true;
    // ROS_INFO("Serial::set_pos_read ends");
}
void Serial::serial_write(std::string cmd)
{
    start = clock();
    ser.write(cmd);
    //serial_write(cmd);
    //serial.serial_write(set_pos_cmd);
}
void Serial::ros_loop(uint16_t source_id, uint16_t node_id, float &current_pos, float desired_pos)
{
    //float current_pos;
    // uint16_t source_id = 1;
    //uint16_t node_id = 100;
    //float desired_pos = -100.0;
    //std::vector<float> joint_position_;
    ros::Rate loop_rate(1000);
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    //std::ios_base::sync_with_stdio(false);

    if (init()) {
        while (ros::ok()) {
            //motor_pos_read(current_pos);
            //serial::motor_pos_read();
            //joint_pos.data[0].push_back(current_pos);

            serial_write(set_pos_cmd(source_id, node_id, 500.0));
            //motor_pos_write(get_pos_cmd(source_id, node_id));
            //motor_pos_read(current_pos, 0.005);
            double five_ms = .005;
            //bool response = motor_pos_read(current_pos, five_ms); //rad && sec
            set_pos_read(five_ms);
            serial_write(get_pos_cmd(source_id, node_id));
            //motor_pos_write(get_pos_cmd(source_id, node_id));
            //motor_pos_read(current_pos, 0.005);
            //bool response = motor_pos_read(current_pos, five_ms); //rad && sec
            get_pos_read(current_pos, five_ms);
            //end = clock();
            //double elasped_time = ((double) (end - start)) / CLOCKS_PER_SEC;
            //std::ios_base::sync_with_stdio(false);
            //std::cout << "  start_time is : " << start << '\n';
            //std::cout << "  end_time is : " << end << '\n';
            //std::cout << "  elapsed_time is : " << elasped_time * 1000 << " msec" << '\n';
            //ROS_INFO("< 5 ms: %s", response ? "true" : "false");
            // serial::motor_pos_write(set_pos_cmd);
            loop_rate.sleep();
            //ROS_INFO(" node started");
            //ros::spinOnce();
        }
        // ros::shutdown();
    }

    //ros::spin();
}
