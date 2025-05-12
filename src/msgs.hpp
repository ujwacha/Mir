struct Twist_msg {
  float x;
  float y;
  float z;
};

struct ImuData {
  double yaw;
  double pitch;
  double roll;
  float accel_x;
  float accel_y;
  float accel_z;
};

struct OdomMsg {
  float omega_l;
  float omega_r;
  float omega_m;
};

struct Distance_Sensors {
  float d_mid;
  uint16_t s_mid;
  float d_right;
  uint16_t s_right;
  float d_left;
  uint16_t s_left;
};


struct Sick {
  float d_one;
  float d_two;
  float d_three;
  float d_four;
};
