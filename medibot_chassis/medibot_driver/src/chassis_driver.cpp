#include "medibot_driver/chassis_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "medibot_driver/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "medibot_msgs/msg/data.hpp"

using std::placeholders::_1;
using namespace std;

void sigintHandler(int sig);
sensor_msgs::msg::Imu Mpu6050;
rclcpp::Node::SharedPtr node_handle = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    MedibotDriver driver;
    driver.Run();
    rclcpp::shutdown();
    return 0;
}

short MedibotDriver::DecodeIMU(uint8_t high, uint8_t low) {
    short val = 0;
    val |= high << 8;
    val |= low;
    return val;
}

float MedibotDriver::DecodeOdom(uint8_t high, uint8_t low) {
    float result;
    short raw = 0;
    raw |= high << 8;
    raw |= low;
    result = (raw / 1000) + (raw % 1000) * 0.001;
    return result;
}

void MedibotDriver::on_ackermann(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm) {
    short val;

    tx_data_.bytes[0] = FRAME_HEAD;
    tx_data_.bytes[1] = 0;
    tx_data_.bytes[2] = 0;

    val = akm->drive.speed * 1000;
    tx_data_.bytes[4] = val;
    tx_data_.bytes[3] = val >> 8;

    val = akm->drive.steering_angle * 1000 / 2;
    tx_data_.bytes[8] = val;
    tx_data_.bytes[7] = val >> 8;

    tx_data_.bytes[9] = ComputeChecksum(9, DATA_CHECK);
    tx_data_.bytes[10] = FRAME_TAIL;

    try {
        stm32_port.write(tx_data_.bytes, sizeof(tx_data_.bytes));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to write serial data");
    }
}

void MedibotDriver::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr twist) {
    short val;
    tx_data_.bytes[0] = FRAME_HEAD;
    tx_data_.bytes[1] = 0;
    tx_data_.bytes[2] = 0;

    val = twist->linear.x * 1000;
    tx_data_.bytes[4] = val;
    tx_data_.bytes[3] = val >> 8;

    val = twist->linear.y * 1000;
    tx_data_.bytes[6] = val;
    tx_data_.bytes[5] = val >> 8;

    val = twist->angular.z * 1000;
    tx_data_.bytes[8] = val;
    tx_data_.bytes[7] = val >> 8;

    tx_data_.bytes[9] = ComputeChecksum(9, DATA_CHECK);
    tx_data_.bytes[10] = FRAME_TAIL;

    try {
        if (ack_topic_ == "none") {
            stm32_port.write(tx_data_.bytes, sizeof(tx_data_.bytes));
        }
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to write serial data");
    }
}

void MedibotDriver::on_sign_switch(const std_msgs::msg::Int32::SharedPtr msg) {
    (void)msg;
}

void MedibotDriver::PublishImu() {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Node::now();
    imu_msg.header.frame_id = gyro_frame_;

    imu_msg.orientation.x = Mpu6050.orientation.x;
    imu_msg.orientation.y = Mpu6050.orientation.y;
    imu_msg.orientation.z = Mpu6050.orientation.z;
    imu_msg.orientation.w = Mpu6050.orientation.w;
    imu_msg.orientation_covariance[0] = 1e6;
    imu_msg.orientation_covariance[4] = 1e6;
    imu_msg.orientation_covariance[8] = 1e-6;

    imu_msg.angular_velocity.x = Mpu6050.angular_velocity.x;
    imu_msg.angular_velocity.y = Mpu6050.angular_velocity.y;
    imu_msg.angular_velocity.z = Mpu6050.angular_velocity.z;
    imu_msg.angular_velocity_covariance[0] = 1e6;
    imu_msg.angular_velocity_covariance[4] = 1e6;
    imu_msg.angular_velocity_covariance[8] = 1e-6;

    imu_msg.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    imu_msg.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    imu_msg.linear_acceleration.z = Mpu6050.linear_acceleration.z;

    imu_pub_->publish(imu_msg);
}

void MedibotDriver::PublishOdometry() {
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_.theta);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    medibot_msgs::msg::Data pose_msg;
    medibot_msgs::msg::Data vel_msg;
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vel_.x;
    odom.twist.twist.linear.y = vel_.y;
    odom.twist.twist.angular.z = vel_.theta;

    pose_msg.x = pose_.x;
    pose_msg.y = pose_.y;
    pose_msg.z = pose_.theta;

    vel_msg.x = vel_.x;
    vel_msg.y = vel_.y;
    vel_msg.z = vel_.theta;

    odom_pub_->publish(odom);
    pose_pub_->publish(pose_msg);
    vel_pub_->publish(vel_msg);
}

void MedibotDriver::PublishBattery() {
    std_msgs::msg::Float32 batt_msg;
    static int pub_counter = 0;

    if (pub_counter++ > 10) {
        pub_counter = 0;
        batt_msg.data = battery_voltage_;
        battery_pub_->publish(batt_msg);
    }
}

unsigned char MedibotDriver::ComputeChecksum(unsigned char count, unsigned char mode) {
    unsigned char check = 0, k;

    if (mode == 0) {
        for (k = 0; k < count; k++) {
            check = check ^ rx_data_.bytes[k];
        }
    } else if (mode == 1) {
        for (k = 0; k < count; k++) {
            check = check ^ tx_data_.bytes[k];
        }
    }

    return check;
}

bool MedibotDriver::ReadSensorData() {
    short raw_val = 0, j = 0, head_pos = 0, tail_pos = 0;
    uint8_t raw_buf[RECV_BUF_SIZE] = {0};
    stm32_port.read(raw_buf, sizeof(raw_buf));

    for (j = 0; j < 24; j++) {
        if (raw_buf[j] == FRAME_HEAD)
            head_pos = j;
        else if (raw_buf[j] == FRAME_TAIL)
            tail_pos = j;
    }

    if (tail_pos == (head_pos + 23)) {
        memcpy(rx_data_.bytes, raw_buf, sizeof(rx_data_.bytes));
    } else if (head_pos == (1 + tail_pos)) {
        for (j = 0; j < 24; j++)
            rx_data_.bytes[j] = raw_buf[(j + head_pos) % 24];
    } else {
        return false;
    }

    rx_data_.head = rx_data_.bytes[0];
    rx_data_.tail = rx_data_.bytes[23];

    if (rx_data_.head == FRAME_HEAD) {
        if (rx_data_.tail == FRAME_TAIL) {
            if (rx_data_.bytes[22] == ComputeChecksum(22, RECV_CHECK) || (head_pos == (1 + tail_pos))) {
                rx_data_.stop_flag = rx_data_.bytes[1];
                vel_.x = DecodeOdom(rx_data_.bytes[2], rx_data_.bytes[3]);
                vel_.y = DecodeOdom(rx_data_.bytes[4], rx_data_.bytes[5]);
                vel_.theta = DecodeOdom(rx_data_.bytes[6], rx_data_.bytes[7]);

                imu_raw_.accel_x = DecodeIMU(rx_data_.bytes[8], rx_data_.bytes[9]);
                imu_raw_.accel_y = DecodeIMU(rx_data_.bytes[10], rx_data_.bytes[11]);
                imu_raw_.accel_z = DecodeIMU(rx_data_.bytes[12], rx_data_.bytes[13]);
                imu_raw_.gyro_x = DecodeIMU(rx_data_.bytes[14], rx_data_.bytes[15]);
                imu_raw_.gyro_y = DecodeIMU(rx_data_.bytes[16], rx_data_.bytes[17]);
                imu_raw_.gyro_z = DecodeIMU(rx_data_.bytes[18], rx_data_.bytes[19]);

                Mpu6050.linear_acceleration.x = imu_raw_.accel_x / ACCEL_SCALE;
                Mpu6050.linear_acceleration.y = imu_raw_.accel_y / ACCEL_SCALE;
                Mpu6050.linear_acceleration.z = imu_raw_.accel_z / ACCEL_SCALE;

                Mpu6050.angular_velocity.x = imu_raw_.gyro_x * GYRO_SCALE;
                Mpu6050.angular_velocity.y = imu_raw_.gyro_y * GYRO_SCALE;
                Mpu6050.angular_velocity.z = imu_raw_.gyro_z * GYRO_SCALE;

                raw_val = 0;
                raw_val |= rx_data_.bytes[20] << 8;
                raw_val |= rx_data_.bytes[21];
                battery_voltage_ = raw_val / 1000 + (raw_val % 1000) * 0.001;

                return true;
            }
        }
    }

    return false;
}

void MedibotDriver::Run() {
    rclcpp::Time cur_time, prev_time;
    cur_time = rclcpp::Node::now();
    prev_time = rclcpp::Node::now();
    while (rclcpp::ok()) {
        cur_time = rclcpp::Node::now();
        dt_ = (cur_time - prev_time).seconds();
        if (ReadSensorData()) {
            pose_.x += 1.03 * (vel_.x * cos(pose_.theta) - vel_.y * sin(pose_.theta)) * dt_;
            pose_.y += 1.125 * (vel_.x * sin(pose_.theta) + vel_.y * cos(pose_.theta)) * dt_;
            pose_.theta += vel_.theta * dt_;

            Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                      Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
            PublishImu();
            PublishBattery();
            PublishOdometry();
            rclcpp::spin_some(this->get_node_base_interface());
        }
        prev_time = cur_time;
    }
}

MedibotDriver::MedibotDriver()
    : rclcpp::Node("medibot_driver") {
    memset(&pose_, 0, sizeof(pose_));
    memset(&vel_, 0, sizeof(vel_));
    memset(&rx_data_, 0, sizeof(rx_data_));
    memset(&tx_data_, 0, sizeof(tx_data_));
    memset(&imu_raw_, 0, sizeof(imu_raw_));

    int serial_baud = 115200;

    this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
    this->declare_parameter<std::string>("cmd_vel", "medibot_cmd_vel");
    this->declare_parameter<std::string>("akm_cmd_vel", "medibot_ackermann_cmd");
    this->declare_parameter<std::string>("odom_frame_id", "medibot_odom");
    this->declare_parameter<std::string>("robot_frame_id", "medibot_base_link");
    this->declare_parameter<std::string>("gyro_frame_id", "medibot_gyro_link");

    this->get_parameter("serial_baud_rate", serial_baud);
    this->get_parameter("usart_port_name", port_name_);
    this->get_parameter("cmd_vel", cmd_topic_);
    this->get_parameter("akm_cmd_vel", ack_topic_);
    this->get_parameter("odom_frame_id", odom_frame_);
    this->get_parameter("robot_frame_id", base_frame_);
    this->get_parameter("gyro_frame_id", gyro_frame_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("medibot_odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("medibot_imu/data_raw", 10);
    battery_pub_ = create_publisher<std_msgs::msg::Float32>("medibot_power_voltage", 1);
    pose_pub_ = create_publisher<medibot_msgs::msg::Data>("medibot_robotpose", 10);
    vel_pub_ = create_publisher<medibot_msgs::msg::Data>("medibot_robotvel", 10);

    tf_bro_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic_, qos, std::bind(&MedibotDriver::on_cmd_vel, this, _1));
    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        ack_topic_, qos, std::bind(&MedibotDriver::on_ackermann, this, _1));

    try {
        stm32_port.setPort("/dev/ttyACM0");
        stm32_port.setBaudrate(serial_baud);
        serial::Timeout tm = serial::Timeout::simpleTimeout(2000);
        stm32_port.setTimeout(tm);
        stm32_port.open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open serial port, please check cable");
    }
    if (stm32_port.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    }
}

void sigintHandler(int sig) {
    sig = sig;
    printf("Medibot driver shutting down...\n");
    serial::Serial stm32;
    stm32.setPort("/dev/ttyACM0");
    stm32.setBaudrate(115200);
    serial::Timeout tm = serial::Timeout::simpleTimeout(2000);
    stm32.setTimeout(tm);
    stm32.open();
    TxFrame stop_frame;
    if (stm32.isOpen()) {
        stop_frame.bytes[0] = FRAME_HEAD;
        stop_frame.bytes[1] = 0;
        stop_frame.bytes[2] = 0;
        stop_frame.bytes[4] = 0;
        stop_frame.bytes[3] = 0;
        stop_frame.bytes[6] = 0;
        stop_frame.bytes[5] = 0;
        stop_frame.bytes[7] = 0;
        stop_frame.bytes[8] = 0;
        int cksum = 0;
        for (int k = 0; k < 9; k++) {
            cksum = cksum ^ stop_frame.bytes[k];
        }
        stop_frame.bytes[9] = cksum;
        stop_frame.bytes[10] = FRAME_TAIL;

        try {
            stm32.write(stop_frame.bytes, sizeof(stop_frame.bytes));
        } catch (serial::IOException& e) {
        }
    }
    rclcpp::shutdown();
}

MedibotDriver::~MedibotDriver() {
    RCLCPP_INFO(this->get_logger(), "Medibot driver destroyed");
}
