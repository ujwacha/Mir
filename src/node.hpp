#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include "msgs.hpp"
#include "KalmanFilter.hpp"


class Sayer : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr odom_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr distance_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr strength_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time time;
  double prev_time;

  Twist_msg twist_msg;
  ImuData imu_data;
  OdomMsg odom_msg;
  Distance_Sensors dis_mrl;

  KalmanFilter Kalman;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


public:
  Sayer(const rclcpp::NodeOptions &options) : Node("MirLocalize", options) {
    const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort().durability_volatile();

    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      currentqol,
      std::bind(
        &Sayer::cmd_vel_subscription_callback, this, std::placeholders::_1
      )
    );


    rpy_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "imu/rpy",
      currentqol,
      std::bind(
        &Sayer::rpy_callback, this, std::placeholders::_1
      )
    );


    distance_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/distance_mrl/length",
      currentqol,
      std::bind(
        &Sayer::distance_callback, this, std::placeholders::_1
      )
    );

    strength_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/distance_mrl/strength",
      currentqol,
      std::bind(
        &Sayer::strength_callback, this, std::placeholders::_1
      )
    );




    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw",
      currentqol,
      std::bind(&Sayer::imu_subscription_callback, this, std::placeholders::_1)
    );

    odom_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
      "odom_vec",
      currentqol,
      std::bind(&Sayer::odom_subscription_callback, this, std::placeholders::_1)
    );

    timer = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&Sayer::update_kalman_data, this)
    );
    

    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);


    prev_time = this->now().seconds();
    // initial_time = time.n();
  }

  void cmd_vel_subscription_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg
  ) {
    twist_msg.x = msg->linear.x;
    twist_msg.y = msg->linear.y;
    twist_msg.z = msg->angular.z;
  }

  void rpy_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg
  ) {
    imu_data.roll = msg->x;
    imu_data.pitch = msg->y;
    imu_data.yaw = msg->z;
  }

  void distance_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    dis_mrl.d_mid = msg->x;
    dis_mrl.d_right = msg->y;
    dis_mrl.d_left = msg->z;
  }

void strength_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg
  ) {
    dis_mrl.s_mid = msg->x;
    dis_mrl.s_right = msg->y;
    dis_mrl.s_left = msg->z;
  }


  void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto &orientation = msg->orientation;

    tf2::Quaternion quat(
      orientation.x, orientation.y, orientation.z, orientation.w
    );

//    tf2::Matrix3x3(quat).getRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);

    const auto &acceleration = msg->linear_acceleration;
    imu_data.accel_x = acceleration.x;
    imu_data.accel_y = acceleration.y;
    imu_data.accel_z = acceleration.z;
  }

  void odom_subscription_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_msg.omega_l = msg->x;
    odom_msg.omega_r = msg->y;
    odom_msg.omega_m = msg->z;
  }

  void update_kalman_data() {

    double time = this->now().seconds() - prev_time;

    Twist_msg msg;
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;

    Kalman.predict(msg, time);

    Kalman.wheel_update(odom_msg, time);
    Kalman.imu_update(imu_data, time);
    Kalman.distance_update(dis_mrl, time);

    prev_time = this->now().seconds();
  }

  void publish() {
    // Create Odometry message
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // Calculate elapsed time
    const auto current_time = this->get_clock()->now();
    // double t = seconds_since_start(current_time);

    // Circular motion with radius 0.5m
    const double radius = 0.5;
    msg.pose.pose.position.x = Kalman.state[State::X];
    msg.pose.pose.position.y = Kalman.state[State::Y];
    msg.pose.pose.position.z = 0.0;

    // Orientation quaternion (rotation around z-axis)
    double theta = Kalman.state[State::THETA];

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(theta / 2.0);
    msg.pose.pose.orientation.w = std::cos(theta / 2.0);

    // Velocity components
    double vx = Kalman.state[State::VX];
    double vy = Kalman.state[State::VY];

    // frame con
    msg.twist.twist.linear.x = vx * std::cos(theta) + vy * std::sin(theta);
    
    msg.twist.twist.linear.y = -vx * std::sin(theta) + vy * std::cos(theta);
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.z = Kalman.state[State::OMEGA];

    // Covariance matrices (set to zero for simplicity)
    for (size_t i = 0; i < 36; ++i) {
      msg.pose.covariance[i] = 0.0;
      msg.twist.covariance[i] = 0.0;
    }

    auto state_cov = Kalman.ekf.getCovariance();

    msg.pose.covariance[1] = state_cov(State::X, State::X);
    msg.pose.covariance[2] = state_cov(State::Y, State::Y);

    odom_publisher->publish(msg);

  }
};
