#include <PS5.hpp>
#include "netlib.hpp"
#include <iostream>

using namespace std;
using namespace pad;
using namespace netlib;

const float   INT16_MAX_VAL = 32768;
const int16_t HIGH_SPEED_LIMIT   = 1500;
const int16_t NORMAL_SPEED_LIMIT = 1200;
const int16_t SLOW_SPEED_LIMIT   = 850;
const int16_t UP_SPEED   = -1300;
const int16_t DOWN_SPEED = 350;
const int16_t Z_BRAKE_SPEED = -50;
const uint8_t ARG_MIN = 0;
const uint8_t ARG_MAX = 150;
const uint8_t ARG_INITIAL = 75;
const float   ARG_DIFF = 0.15;

enum HandleMode {
  Normal,
  Slow,
  High,
  Stop
};

struct ETH_TxDataType {
  int16_t x_speed;
  int16_t y_speed;
  int16_t z_speed;
  uint8_t arm_state;
};

int16_t set_speed(int16_t data, HandleMode mode, float ratio) {
  if (data == 0) return 0;

  uint16_t limit;

  switch (mode) {
    case Normal: {
      limit = NORMAL_SPEED_LIMIT;
      break;
    }
    case High: {  
      limit = HIGH_SPEED_LIMIT;
      break;
    }
    case Slow: {  
      limit = SLOW_SPEED_LIMIT;
      break;
    }
    case Stop:   return 0;
  }

  float speed = 0;
  speed = ( data * ( limit / INT16_MAX_VAL ) ) * ratio;

  return static_cast<int16_t>(speed);
}

float set_servo_value(float servo_arg, float diff) {
  float new_arg = servo_arg + diff;

  if (new_arg > ARG_MAX || new_arg < ARG_MIN) {
    return servo_arg;
  }

  return new_arg;
}

int main () {

  ps5::DualSense ps5(ps5::Connect::USB);
  UDPSender pc("192.168.0.111", 11111);

  ETH_TxDataType TxData;

  /* Button & Stick Setting */

  Button& propeller_switch = ps5.Circle.get();
  Button& accel = ps5.R2.get();
  Button& brake = ps5.L2.get();
  Button& stop  = ps5.Cross.get();
  Button& servo_cw  = ps5.R1.get();
  Button& servo_ccw = ps5.L1.get(); 

  Axis& x = ps5.Lstick.x.get();
  Axis& y = ps5.Lstick.y.get();
 
  /* send data */
  int16_t x_data, y_data, z_data = 0; 
  uint8_t servo_data = 0;
  float servo_arg = ARG_INITIAL;
  float diff = 0;

  /* motor speed & state setting */

  float x_ratio = -1.0;
  float y_ratio = -1.0; 
  bool rotate = false;

  /* othre vals */

  HandleMode mode;

  /* end other vals*/

  if (!ps5.isConnected()) return 1;

  while (1) {
    ps5.update();
    if (ps5.Option.pushed()) break;
  }

  while (ps5.isConnected()) {
    ps5.update();

    /* XY speed mode setting */
    if (stop.pressed())
      mode = Stop;
    else if (brake.pressed())      
      mode = Slow;
    else if (accel.pressed()) 
      mode = High;
    else 
      mode = Normal;


    /* Z speed setting */
    if (ps5.Up.pressed()) {
      z_data = UP_SPEED;
    }
    else if (ps5.Down.pressed()) {
      z_data = DOWN_SPEED;
    }
    else {
      z_data = Z_BRAKE_SPEED;
    }

    /* servo arg setting */

    if (servo_cw.pressed()) {
      diff = ARG_DIFF;
    }
    else if (servo_ccw.pressed()) {
      diff = -ARG_DIFF;
    }
    else {
      diff = 0;
    }

    servo_arg = set_servo_value(servo_arg, diff);
    servo_data = static_cast<uint8_t>(servo_arg);

    /* propeller state setting */
    if (propeller_switch.pushed()) {
      rotate = !rotate;
    }

  
    /* send data setting */
    TxData.x_speed = set_speed(x.getValue(), mode, x_ratio);
    TxData.y_speed = set_speed(y.getValue(), mode, y_ratio);
    TxData.z_speed = z_data;
    TxData.arm_state = (rotate << 7) | servo_data;

    string mode_msg;
    switch (mode) {
      case Normal: {
        mode_msg = "normal";
        break;
      }
      case High: {
        mode_msg = "high";
        break;
      }
      case Slow: {
        mode_msg = "slow";
        break;
      }
      case Stop: {
        mode_msg = "stop";
        break;
      }
    }

    cout << "[ mode: " << mode_msg << " ] " << "x: " << TxData.x_speed << " y: " << TxData.y_speed << 
      " z: " << TxData.z_speed << " arm: " << (int)(TxData.arm_state & 0x7f) << 
      " rotate: " << (int)(TxData.arm_state >> 7 & 0x01) << endl;

    pc.send(TxData);

    usleep(2500);

    if (ps5.Option.pushed()) break;
  }

  return 0;
}