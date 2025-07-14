#include <cmath>
#include <tuple>
#include <iostream>

template <typename T> class RobotSensorKalman {
private:
  T l, b, r, sensor_angle, beam_angle;       // Environment and robot parameters
  const T eps = 1e-6; // For numerical stability

  // Structure to hold both distance and derivatives
  struct SensorResult {
    T distance;
    T dx;     // ∂distance/∂x
    T dy;     // ∂distance/∂y
    T dtheta; // ∂distance/∂theta_s
  };

  T normalizeAngle(T angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

public:
  RobotSensorKalman(T angle_to_cir, T radius_to_cir, T angle_bea, T length , T breadth)
    : l(length), b(breadth), r(radius_to_cir), sensor_angle(angle_to_cir), beam_angle(angle_bea) {}

  SensorResult calculate(T x, T y, T theta_yaw) {
    SensorResult result;

    T theta_beam = normalizeAngle(theta_yaw + beam_angle);
    T theta_sensor = normalizeAngle(theta_yaw + sensor_angle);

    // from this we need to change the values of x and y
    // to represent the place where x and y exist

    T x_sensor = x + r*std::cos(theta_sensor);
    T y_sensor = y + r*std::sin(theta_sensor);

    // print_res(result);


    // std::cout << "Sensor{l: " << l << ", b: " << b << ", r: " << r << ", a: " << a <<  " } : " <<  std::endl;

    // std::cout << "DISTANCE{x: " << x << ", y: " << y << ", theta_s: " << theta_s << " } : " <<  std::endl;

    //Calculate angles to boundaries
    const T theta = std::atan2(y_sensor, x_sensor);
    const T beta = std::atan2(y_sensor, b - x_sensor);
    const T alpha = std::atan2(l - y_sensor, b - x_sensor);
    const T gamma = std::atan2(l - y_sensor, x_sensor);



    // Determine which edge we're facing
    if (-beta <= theta_beam && theta_beam <= alpha) { // Right edge

      result.distance = (b - x_sensor) / std::cos(theta_beam);
      result.dx = -1.0 / std::cos(theta_beam);
      result.dy = 0.0;
      result.dtheta = (1/std::cos(theta_beam))*(std::tan(theta_beam)*(b - x_sensor) + r*std::sin(theta_sensor));
      
    } else if (alpha <= theta_beam && theta_beam <= M_PI - gamma) { // Top edge

      result.distance = (l - y_sensor) / std::sin(theta_beam);
      result.dx = 0.0;
      result.dy = -1.0 / std::sin(theta_beam);
      result.dtheta = (1/std::sin(theta_beam))*((1/std::tan(theta_beam))*(-l + y_sensor) - r*std::cos(theta_sensor));

    } else if (-M_PI + theta <= theta_beam && theta_beam <= -beta) { // Bottom edge

      result.distance = -y_sensor / std::sin(theta_beam);
      result.dx = 0.0;
      result.dy = -1.0 / std::sin(theta_beam);
      result.dtheta = -(1/std::sin(theta_beam))*(r*std::cos(theta_sensor) - (1/std::tan(theta_beam))*(y_sensor));

    } else { // Left edge

      result.distance = -x_sensor / std::cos(theta_beam);
      result.dx = -1.0 / std::cos(theta_beam);
      result.dy = 0.0;
      result.dtheta = (1/std::cos(theta_beam))*(r*std::sin(theta_sensor) - std::tan(theta_beam)*(x_sensor));

    }

    // print_res(result);

    // Clamp negative distances and handle numerical instability
    if (result.distance < 0) {
      result.distance = 0;
      result.dx = result.dy = result.dtheta = 0;
    }

    return result;
  }

  void print_res(SensorResult s) {
    
    std::cout << "RESULT{dis: " << s.distance << ", dx: " << s.dx << ", dy: " << s.dy << ", dth: " << s.dtheta << " } : " <<  std::endl;
    
  }
};
