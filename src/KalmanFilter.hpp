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


#include <Eigen/Dense>

#include "filter.h"

#define MID_RADIUS 0.235

// #define RIGHT_RADIUS 0.2425
// #define LEFT_RADIUS 0.2425

#define RIGHT_RADIUS 0.32
#define LEFT_RADIUS 0.165

#define WHEEL_D 0.0574
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

  WheelMeasurementModel wheel_model{0, LEFT_RADIUS, 0, 0, -RIGHT_RADIUS, 0, -MID_RADIUS, 0 , PI/2 , WHEEL_D/2};


  //  TFMiniMeasurementModel mini{0, 0.3, PI/2, 0.3, -PI/2, 0.3, 4, 7.5};
  //TFMiniMeasurementModel mini(PI/2, 0.2, 0, 0.2, -PI/2, 0.2, 4, 7.5);
  TwoSensorsMeasurementModel sensor_left{0, 0.3, PI/2, 0.3, 4, 7.5};
  TwoSensorsMeasurementModel sensor_right{0, 0.3, -PI/2, 0.3, 4, 7.5};


  Kalman::Covariance<ImuMeasurement> imu_cov;
  Kalman::Covariance<WheelMeasurement> wheel_cov;
  Kalman::Covariance<State> state_cov;
  // Kalman::Covariance<TFMiniMeasurement> tf_cov;
  Kalman::Covariance<TwoSensorsMeasurement> cov_right;
  Kalman::Covariance<TwoSensorsMeasurement> cov_left;




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
    state_cov /= 100; // for now, must change later

    state_cov(State::VX, State::VX) = 2;
    state_cov(State::VY, State::VY) = 2;
    state_cov(State::OMEGA, State::OMEGA) = 0.5;
    state_cov(State::AX, State::AX) = 4;
    state_cov(State::AY, State::AY) = 4;


    // state_cov(State::VX, State::VX) = 1;
    // state_cov(State::VY, State::VY) = 1;
    // state_cov(State::OMEGA, State::OMEGA) = 1;

    wheel_cov(WheelMeasurement::OMEGA_L, WheelMeasurement::OMEGA_L) = 1;
    wheel_cov(WheelMeasurement::OMEGA_R, WheelMeasurement::OMEGA_R) = 1;
    wheel_cov(WheelMeasurement::OMEGA_M, WheelMeasurement::OMEGA_M) = 1;


    // tf_cov(TFMiniMeasurement::D1, TFMiniMeasurement::D1) = 0.1;
    // tf_cov(TFMiniMeasurement::D2, TFMiniMeasurement::D2) = 0.1;
    // tf_cov(TFMiniMeasurement::D3, TFMiniMeasurement::D3) = 0.1;

    cov_right.setIdentity();
    cov_left.setIdentity();


    imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 1;
    imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 1;
    imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.001;


    imu_model.setCovariance(imu_cov);
    wheel_model.setCovariance(wheel_cov);
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

  void wheel_update(OdomMsg msg, double time) {
    WheelMeasurement wheel;

    double omega_r = omega_r_lpf.update(msg.omega_r);
    double omega_l = omega_l_lpf.update(msg.omega_l);
    double omega_m = omega_m_lpf.update(msg.omega_m);
    
    wheel.omega_r() = omega_r;
    wheel.omega_l() = omega_l;
    wheel.omega_m() = omega_m;
    
    x_ekf = ekf.update(wheel_model, wheel, time);
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

  void distance_update(Distance_Sensors msg, double time) {
    // TFMiniMeasurement min;
    // min.d1() = msg.d_mid/100;
    // min.d3() = msg.d_left/100;
    // min.d2() = msg.d_right/100;

    // tf_cov(TFMiniMeasurement::D1, TFMiniMeasurement::D1) = get_cov(msg.s_mid);
    // tf_cov(TFMiniMeasurement::D3, TFMiniMeasurement::D3) = get_cov(msg.s_left);
    // tf_cov(TFMiniMeasurement::D2, TFMiniMeasurement::D2) = get_cov(msg.s_right);
    
    // mini.setCovarianceSquareRoot(tf_cov);
    
    // x_ekf = ekf.update(mini, min, time);

    
    TwoSensorsMeasurement right;
    TwoSensorsMeasurement left;


    // min.d1() = dm[i]/100;
    // min.d3() = dl[i]/100;
    // min.d2() = dr[i]/100;

    right.d1() = msg.d_mid/100;
    right.d2() = msg.d_right/100;
      
    left.d1() = msg.d_mid/100;
    left.d2() = msg.d_left/100;
      
      


    cov_right(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) = 0.09;
    cov_right(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) = 0.09;

    cov_left(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) = 0.09;
    cov_left(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) = 0.09;

    sensor_right.setCovarianceSquareRoot(cov_right);
    sensor_left.setCovarianceSquareRoot(cov_left);


    // std::cout << "REAL d1 : " << min.d1() << std::endl
    // 		<< "REAL d2 : " << min.d2() << std::endl
    // 		<< "REAL d3 : " << min.d3() << std::endl;


    //if (i % 300 == 0 || i < 250) 
    // x_ekf = ekf.update(mini, min, time);

    x_ekf = ekf.update(sensor_right, right, time, true, 0.8);
    x_ekf = ekf.update(sensor_left, left, time, true, 1);


  }
};
