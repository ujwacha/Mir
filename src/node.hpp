#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

#include <Eigen/Dense>
#include "msgs.hpp"
#include "KalmanFilter.hpp"
#include "three_wheels_to_twist.hpp"

#include "sick_interfaces/msg/all_sensors.hpp"



#define WHEEL_D 0.0574

class Sayer : public rclcpp::Node {
private:
  rclcpp::Subscription<sick_interfaces::msg::AllSensors>::SharedPtr sensors_sub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr timer_predict;
  double prev_time;

 
  KalmanFilter Kalman;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Create controller instance
  Controller controller_model{
      -8.9/100,  -17.6/100, 0,        // Wheel 1 position and angle
      13.81/100, 0.61/100,  M_PI / 2, // Wheel 2 position and angle
      7.3/100,   16.4/100,  0,        // Wheel 3 position and angle
      WHEEL_D / 2                         // Wheel radius
  };



public:
  Sayer(const rclcpp::NodeOptions &options) : Node("MirLocalize", options) {
    const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort().durability_volatile();
    RCLCPP_INFO(this->get_logger(), "Hi Mir, Kalman Filter");



    sensors_sub = this->create_subscription<sick_interfaces::msg::AllSensors>(
        "sensors", currentqol,
        std::bind(&Sayer::sensors_subscription_callback, this,
                  std::placeholders::_1));

    timer =
        this->create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&Sayer::publish, this));

    odom_publisher =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    prev_time = 0;
    // initial_time = time.n();
  }


  void sensors_subscription_callback(const sick_interfaces::msg::AllSensors msg) {
    if (msg.timestamp_seconds < prev_time) return;

    RCLCPP_INFO(this->get_logger(), "Sensor Msg Recieved");

    double time_difference = msg.timestamp_seconds - prev_time;
    prev_time = msg.timestamp_seconds;


    auto tw = controller_model.get_output(msg.omegas.x,
					  msg.omegas.y,
					  msg.omegas.z);
    
    Twist_msg twist_msg;

    twist_msg.x = tw.vx;
    twist_msg.y = tw.vx;
    twist_msg.z = tw.vx;

    Kalman.predict(twist_msg, time_difference);
    RCLCPP_INFO(this->get_logger(), "Predicted");

    ImuData imu_data;

    imu_data.roll = msg.imu_euler.x;
    imu_data.pitch = msg.imu_euler.y;
    imu_data.yaw = msg.imu_euler.z;
    
    imu_data.accel_x = msg.imu_accel.x;
    imu_data.accel_y = msg.imu_accel.y;
    imu_data.accel_z = msg.imu_accel.z;
    
    Kalman.imu_update(imu_data, time_difference);
    RCLCPP_INFO(this->get_logger(), "Updated IMU");

    if (!msg.is_sick_latest) return;

    Sick sick;


    sick.d_one = msg.sick_data.distance_one/100;
    sick.works_one = msg.sick_data.works_one;

    sick.d_two = msg.sick_data.distance_two/100;
    sick.works_two = msg.sick_data.works_two;

    sick.d_three = msg.sick_data.distance_three/100;
    sick.works_three = msg.sick_data.works_three;

    sick.d_four = msg.sick_data.distance_four/100;
    sick.works_four= msg.sick_data.works_four;


    
    Kalman.distance_update(sick, time_difference);

    RCLCPP_INFO(this->get_logger(), "Updated SICK");
  }


  void publish() {
    // Calculate elapsed time
    const auto current_time = this->get_clock()->now();
    // double t = seconds_since_start(current_time);

    // Orientation quaternion (rotation around z-axis)
    double theta = Kalman.x_ekf[State::THETA];

    // Create quaternion from yaw
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = Kalman.x_ekf[State::X];
    odom_tf.transform.translation.y = Kalman.x_ekf[State::Y];
    ;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = quaternion.x();
    odom_tf.transform.rotation.y = quaternion.y();
    odom_tf.transform.rotation.z = quaternion.z();
    odom_tf.transform.rotation.w = quaternion.w();

    RCLCPP_INFO(this->get_logger(), "Published transform message");
    tf_broadcaster_->sendTransform(odom_tf);

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // Circular motion with radius 0.5m
    msg.pose.pose.position.x = Kalman.x_ekf[State::X];
    msg.pose.pose.position.y = Kalman.x_ekf[State::Y];
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_tf.transform.rotation;

    // Velocity components
    double vx = Kalman.x_ekf[State::VX];
    double vy = Kalman.x_ekf[State::VY];

    // frame con
    // msg.twist.twist.linear.x = vx * std::cos(theta) + vy * std::sin(theta);

    // msg.twist.twist.linear.y = -vx * std::sin(theta) + vy * std::cos(theta);
    // msg.twist.twist.linear.z = 0.0;
    // msg.twist.twist.angular.z = Kalman.x_ekf[State::OMEGA];

    // since it is already relative, the frame conversion is unnecessary

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.z = Kalman.x_ekf[State::OMEGA];


    // Covariance matrices (set to zero for simplicity)
    // send the covariance from kalman filter later to see ellipse in rviz
    for (size_t i = 0; i < 36; ++i) {
      msg.pose.covariance[i] = 0.0;
      msg.twist.covariance[i] = 0.0;
    }

    auto state_cov = Kalman.ekf.getCovariance();

    msg.pose.covariance[0] = state_cov(State::X, State::X);
    msg.pose.covariance[6] = state_cov(State::Y, State::Y);
    odom_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published odometry message");
  }
};
