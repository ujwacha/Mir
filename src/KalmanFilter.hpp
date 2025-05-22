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
//#include "TFMiniMeasurementModek.hpp"
#include "TwoDistanceSensorsMeasurementModel.hpp"



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
// typedef Robot::TFMiniMeasurement<T> TFMiniMeasurement;
typedef Robot::TwoSensorsMeasurement<T> TwoSensorsMeasurement;

typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
typedef Robot::WheelMeasurementModel<T> WheelMeasurementModel;
// typedef Robot::TFMiniMeasurementModel<T> TFMiniMeasurementModel;
typedef Robot::TwoSensorsMeasurementModel<T> TwoSensorsMeasurementModel;



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

  // Encoder<float> enc1(-MID_RADIUS, 0, 3.14145/2, WHEEL_D/2);
  // Encoder<float> enc2(0, LEFT_RADIUS, 0, WHEEL_D/2);
  // Encoder<float> enc3(0, -RIGHT_RADIUS, 0, WHEEL_D/2);

  // WheelMeasurementModel wheel_model{0, LEFT_RADIUS, 0, 0, -RIGHT_RADIUS, 0, -MID_RADIUS, 0 , PI/2 , WHEEL_D/2};

  //  TFMiniMeasurementModel mini{0, 0.3, PI/2, 0.3, -PI/2, 0.3, 4, 7.5};
  //TFMiniMeasurementModel mini(PI/2, 0.2, 0, 0.2, -PI/2, 0.2, 4, 7.5);
  // TwoSensorsMeasurementModel sensor_left{0, 0.3, 0, PI/2, 0.3, PI/2, 4, 7.5};
  // TwoSensorsMeasurementModel sensor_right{0, 0.3, 0, -PI/2, 0.3, -PI/2, 4, 7.5};


  TwoSensorsMeasurementModel sensor_first_quardrant_x_y{0, 28/100, PI/2, 27.3/100, 8, 15};
  TwoSensorsMeasurementModel sensor_second_quardrant_y_min_x{PI/2, 27.3/100, PI, 28.5/100, 8, 15};
  TwoSensorsMeasurementModel sensor_third_quardrant_min_x_min_y{PI, 28.5/100, -PI/2, 27/100, 8, 15};
  TwoSensorsMeasurementModel sensor_fourth_quardrant_x_min_y{-PI/2, 27/100, 0, 28/100, 8, 15};


  Kalman::Covariance<ImuMeasurement> imu_cov;
  Kalman::Covariance<WheelMeasurement> wheel_cov;
  Kalman::Covariance<State> state_cov;
  // Kalman::Covariance<TFMiniMeasurement> tf_cov;
  Kalman::Covariance<TwoSensorsMeasurement> cov_first_quard;
  Kalman::Covariance<TwoSensorsMeasurement> cov_second_quard;
  Kalman::Covariance<TwoSensorsMeasurement> cov_third_quard;
  Kalman::Covariance<TwoSensorsMeasurement> cov_fourth_quard;





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

    state.x() = 1.7;
    state.y() = 2.11;

  

    state_cov.setIdentity();
    state_cov /= 10;

    state_cov(State::X, State::X) = 0.01;
    state_cov(State::Y, State::Y) = 0.01;
    state_cov(State::VX, State::VX) = 0.3;
    state_cov(State::VY, State::VY) = 0.3;
    state_cov(State::OMEGA, State::OMEGA) = 0.3;
    state_cov(State::THETA, State::THETA) = 2;
    state_cov(State::AX, State::AX) = 1;
    state_cov(State::AY, State::AY) = 1;

    wheel_cov(WheelMeasurement::OMEGA_L, WheelMeasurement::OMEGA_L) = 1;
    wheel_cov(WheelMeasurement::OMEGA_R, WheelMeasurement::OMEGA_R) = 1;
    wheel_cov(WheelMeasurement::OMEGA_M, WheelMeasurement::OMEGA_M) = 1;


    // tf_cov(TFMiniMeasurement::D1, TFMiniMeasurement::D1) = 0.1;
    // tf_cov(TFMiniMeasurement::D2, TFMiniMeasurement::D2) = 0.1;
    // tf_cov(TFMiniMeasurement::D3, TFMiniMeasurement::D3) = 0.1;

    // cov_right.setIdentity();
    // cov_left.setIdentity();


    imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 1;
    imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 1;
    imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.001;


    imu_model.setCovariance(imu_cov);
    //    wheel_model.setCovariance(wheel_cov);
    //mini.setCovariance(tf_cov);
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

  // void wheel_update(OdomMsg msg, double time) {
  //   WheelMeasurement wheel;

  //   double omega_r = omega_r_lpf.update(msg.omega_r);
  //   double omega_l = omega_l_lpf.update(msg.omega_l);
  //   double omega_m = omega_m_lpf.update(msg.omega_m);
    
  //   wheel.omega_r() = omega_r;
  //   wheel.omega_l() = omega_l;
  //   wheel.omega_m() = omega_m;
    
  //   x_ekf = ekf.update(wheel_model, wheel, time);
  // }

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
    
    TwoSensorsMeasurement first_quardrant;
    TwoSensorsMeasurement second_quardrant;
    TwoSensorsMeasurement third_quardrant;
    TwoSensorsMeasurement fourth_quardrant;

    cov_first_quard(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) =
        0.008;
    cov_first_quard(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) =
        0.008;

    cov_second_quard(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) =
        0.008;
    cov_second_quard(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) =
        0.008;

    cov_third_quard(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) =
        0.008;
    cov_third_quard(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) =
        0.008;

    cov_fourth_quard(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) =
        0.008;
    cov_fourth_quard(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) =
        0.008;

    // convert to meter from centimeter
    first_quardrant.d1() = msg.d_one/100;
    first_quardrant.d2() = msg.d_two/100;

    second_quardrant.d1() = msg.d_two/100;
    second_quardrant.d2() = msg.d_three/100;

    third_quardrant.d1() = msg.d_three/100;
    third_quardrant.d2() = msg.d_four/100;

    fourth_quardrant.d1() = msg.d_four/100;
    fourth_quardrant.d2() = msg.d_one/100;

    if (msg.works_one && msg.works_two)
      x_ekf = ekf.update(sensor_first_quardrant_x_y, first_quardrant, time);

    if (msg.works_two && msg.works_three)
      x_ekf = ekf.update(sensor_second_quardrant_y_min_x, second_quardrant, time);

    if (msg.works_three && msg.works_four)
      x_ekf = ekf.update(sensor_third_quardrant_min_x_min_y, third_quardrant, time);

    if (msg.works_four && msg.works_one)
      x_ekf = ekf.update(sensor_fourth_quardrant_x_min_y, fourth_quardrant, time);

  }
};
