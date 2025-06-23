// This class will have everything that was done in the experiment

#include <iostream>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <vector>
#include <cmath>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/Types.hpp"
#include "ThreeWheel.hpp"
#include "TFMiniMeasurementModek.hpp"
#include "TwoDistanceSensorsMeasurementModel.hpp"
#include "FourDistanceSensorsModel.hpp"
#include "DistanceSensorModel.hpp"



#include "filter.h"

#define MID_RADIUS 0.235

// #define RIGHT_RADIUS 0.2425
// #define LEFT_RADIUS 0.2425

#define RIGHT_RADIUS 0.32
#define LEFT_RADIUS 0.165

//#define WHEEL_D 0.0474


typedef double T;

typedef Robot::State<T> State;
typedef Robot::Twist<T> Twist;
typedef Robot::SystemModel<T> SystemModel;

typedef Robot::OdomMeasurement<T> OdomMeasurement;
typedef Robot::ImuMeasurement<T> ImuMeasurement;
typedef Robot::WheelMeasurement<T> WheelMeasurement;
typedef Robot::TFMiniMeasurement<T> TFMiniMeasurement;
typedef Robot::DistanceSensorMeasurement<T> DistanceSensorMeasurement;
typedef Robot::TwoSensorsMeasurement<T> TwoSensorsMeasurement;
typedef Robot::FourSensorsMeasurement<T> FourSensorsMeasurement;


typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
typedef Robot::WheelMeasurementModel<T> WheelMeasurementModel;
typedef Robot::TFMiniMeasurementModel<T> TFMiniMeasurementModel;
typedef Robot::DistanceSensorMeasurementModel<T> DistanceSensorMeasurementModel;
typedef Robot::TwoSensorsMeasurementModel<T> TwoSensorsMeasurementModel;
typedef Robot::FourDistanceSensorsMeasumementModel<T> FourDistanceSensorsMeasumementModel;

// #define SENSOR_ONE_RADIUS 0.295
// #define SENSOR_TWO_RADIUS 0.273
// #define SENSOR_THREE_RADIUS 0.275
// #define SENSOR_FOUR_RADIUS 0.265

// #define SENSOR_ONE_ANGLE 0
// #define SENSOR_TWO_ANGLE 3.141592 / 2
// #define SENSOR_THREE_ANGLE 3.141592
// #define SENSOR_FOUR_ANGLE -3.141592 / 2



#include "DynamicDistanceModel.hpp"




class KalmanFilter {


private:


  template <typename T>
  T degreesToRadians(T degrees) {
    return degrees * static_cast<T>(M_PI) / static_cast<T>(180);
  }




  // Function to rotate a vector using roll (X-axis) and pitch (Y-axis)
  Eigen::Vector3d rotateVector(const Eigen::Vector3d &v, double roll, double pitch) {
    // Create roll rotation matrix (rotation about X-axis)
    Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    // Create pitch rotation matrix (rotation about Y-axis)
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    
    // Combine the rotations: roll first, then pitch
    Eigen::Matrix3d R = R_pitch * R_roll;
    
    // Apply the rotation to the vector
    return R * v;
  }


  double get_cov(double val) {
    // if (val != 70)
    //   return (1700 * 0.4 /(val - 70));
    // else return 100;

    if (val < 1000) return 70;
    //  else return 0.09;
    else return 0.09;
  }

  Twist twist;

  SystemModel sys;
  ImuMeasurementModel imu_model;


  DistanceSensorMeasurementModel sensor_one{SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, 8.0, 15.0};
  DistanceSensorMeasurementModel sensor_two{SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, 8, 15};
  DistanceSensorMeasurementModel sensor_three{SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, 8, 15};
  DistanceSensorMeasurementModel sensor_four{SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, 8, 15};




  Kalman::Covariance<ImuMeasurement> imu_cov;
  Kalman::Covariance<WheelMeasurement> wheel_cov;
  Kalman::Covariance<State> state_cov;




  // Set Up 1st order and 2nd order low pass filters
  // As and Bs are calculated from GNU octave
  // Use GNU octave signal package and butter function
  double omega_bs[2] = {0.086364, 0.086364};
  double omega_as[2] = {1.0000, -0.8273};
  filter<1> omega_l_lpf{omega_bs, omega_as};
  filter<1> omega_r_lpf{omega_bs, omega_as};
  filter<1> omega_m_lpf{omega_bs, omega_as};
  filter<1> yaw_lpf{omega_bs, omega_as};

  double accel_bs[3] = {9.4469e-04, 1.8894e-03, 9.4469e-04};
  double accel_as[3] = {1.0000, -1.9112, 0.9150};
  filter<2> accel_x_lpf{accel_bs, accel_as};
  filter<2> accel_y_lpf{accel_bs, accel_as};
  filter<2> accel_z_lpf{accel_bs, accel_as};




public:

  State state;
  Kalman::ExtendedKalmanFilter<State> ekf;
  State x_ekf;

  KalmanFilter() {
    state.setZero();

    state.x() = 1;
    state.y() = 1; 
    state.theta() = 0;
    state.yaw_bias() = 0;

    state_cov.setIdentity();
    state_cov /= 10;

    // state_cov(State::X, State::X) = 0.0005;
    // state_cov(State::Y, State::Y) = 0.0005;

    state_cov(State::X, State::X) = 0.0005;
    state_cov(State::Y, State::Y) = 0.0005;

    // state_cov(State::X, State::X) = 1;
    // state_cov(State::Y, State::Y) = 1;

    state_cov(State::VX, State::VX) = 0.3;
    state_cov(State::VY, State::VY) = 0.3;
    state_cov(State::OMEGA, State::OMEGA) = 0.3;
    state_cov(State::THETA, State::THETA) = 0.0005;
    state_cov(State::AX, State::AX) = 1;
    state_cov(State::AY, State::AY) = 1;
    state_cov(State::YAW_BIAS, State::YAW_BIAS) = 1e-6;

    imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 1;
    imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 1;
    imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.00001;

    // set covariance of IMU
    imu_model.setCovariance(imu_cov);

    // Set Covariance Of State Model
    sys.setCovariance(state_cov);

    ekf.init(state);
  }

  void predict(Twist_msg t, double time) {
    Twist twist;

    twist.rvx() = t.x;
    twist.rvy() = t.y;
    twist.romega() = t.z;

    x_ekf = ekf.predict(sys, twist, time);
  }

  void imu_update(ImuData msg, double time) {
    // We can measure the orientation every 5th step
    ImuMeasurement imu;
    // Measurement is affected by noise as well

    double roll = msg.roll;
    double pitch = msg.pitch;
    double yaw = msg.yaw;
    double ax = msg.accel_x;
    double ay = msg.accel_y;
    double az = msg.accel_z;

    Eigen::Vector3d accel = {ax, ay, az};

    Eigen::Vector3d rot_accel = rotateVector(accel, roll, pitch);

    imu.yaw() = yaw;
    imu.ax() = accel_x_lpf.update(rot_accel(0));
    imu.ay() = accel_y_lpf.update(rot_accel(1));

    // Update EK// F
    x_ekf = ekf.update(imu_model, imu, time);
  }

  void distance_update(Sick msg, double time) {
    DistanceSensorMeasurement sensor_one_measurement;
    DistanceSensorMeasurement sensor_two_measurement;
    DistanceSensorMeasurement sensor_three_measurement;
    DistanceSensorMeasurement sensor_four_measurement;

    sensor_one_measurement.d1() = msg.d_one;
    sensor_two_measurement.d1() = msg.d_two;
    sensor_three_measurement.d1() = msg.d_three;
    sensor_four_measurement.d1() = msg.d_four;
    bool use_mahalanobis = true;
    // if (count < 30)
    // 	use_mahalanobis = false;

    std::vector<double> distances;
    unsigned int mask = 0;
    const double single_radius = 1;
    const double radius = 1.4 * single_radius;
    const double divider = 200;

    if (msg.works_one && (ekf.get_mahalanobis(sensor_one, sensor_one_measurement) <
                   single_radius * single_radius)) {
      distances.push_back(sensor_one_measurement.d1());
      mask = mask | SENSOR_ONE;
    }

    if (msg.works_two && (ekf.get_mahalanobis(sensor_two, sensor_two_measurement) <
                   single_radius * single_radius)) {
      distances.push_back(sensor_two_measurement.d1());
      mask = mask | SENSOR_TWO;
    }

    if (msg.works_three && (ekf.get_mahalanobis(sensor_three, sensor_three_measurement) <
                   single_radius * single_radius)) {
      distances.push_back(sensor_three_measurement.d1());
      mask = mask | SENSOR_THREE;
    }

    if (msg.works_four && (ekf.get_mahalanobis(sensor_four, sensor_four_measurement) <
                   single_radius * single_radius)) {
      distances.push_back(sensor_four_measurement.d1());
      mask = mask | SENSOR_FOUR;
    }
    std::cout << "working_distances:" << distances.size() << " mask: " << mask
              << std::endl;
    // std::cin >> aaa;

    if (distances.size() == 1) {

      DistanceSensorMeasurementModel current = get_sensor_one(mask);
      DistanceSensorMeasurement current_measurement;
      current_measurement.d1() = distances[0];
      Kalman::Covariance<DistanceSensorMeasurement> current_one_cov;
      current_one_cov.setIdentity();
      current_one_cov /= divider;
      current.setCovariance(current_one_cov);
      x_ekf = ekf.update(current, current_measurement, time, use_mahalanobis,
                         radius);

    } else if (distances.size() == 2) {

      TwoSensorsMeasurementModel current = get_sensor_two(mask);
      TwoSensorsMeasurement current_measurement;
      current_measurement.d1() = distances[0];
      current_measurement.d2() = distances[1];
      Kalman::Covariance<TwoSensorsMeasurement> current_one_cov;
      current_one_cov.setIdentity();
      current_one_cov /= divider;
      current.setCovariance(current_one_cov);
      x_ekf = ekf.update(current, current_measurement, time, use_mahalanobis,
                         radius);

    } else if (distances.size() == 3) {

      TFMiniMeasurementModel current = get_sensor_three(mask);
      TFMiniMeasurement current_measurement;
      current_measurement.d1() = distances[0];
      current_measurement.d2() = distances[1];
      current_measurement.d3() = distances[2];
      Kalman::Covariance<TFMiniMeasurement> current_one_cov;
      current_one_cov.setIdentity();
      current_one_cov /= divider;
      current.setCovariance(current_one_cov);
      x_ekf = ekf.update(current, current_measurement, time, use_mahalanobis,
                         radius);

    } else if (distances.size() == 4) {

      FourDistanceSensorsMeasumementModel current = get_sensor_four();
      FourSensorsMeasurement current_measurement;
      current_measurement.d1() = distances[0];
      current_measurement.d2() = distances[1];
      current_measurement.d3() = distances[2];
      current_measurement.d4() = distances[3];
      Kalman::Covariance<FourSensorsMeasurement> current_one_cov;
      current_one_cov.setIdentity();
      current_one_cov /= divider;
      current.setCovariance(current_one_cov);
      x_ekf = ekf.update(current, current_measurement, time, use_mahalanobis,
                         radius);
    }
  }
};
