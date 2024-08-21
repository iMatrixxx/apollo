#include "modules/canbus_vehicle/lincoln/protocol/akeman_info_101.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;
using namespace std;

const int32_t AkemanInfo101::ID = 0x101;

void AkemanInfo101::Parse(const std::uint8_t *bytes, int32_t length,
                    Lincoln *chassis_detail) const {

  // double vel_x = parse_two_frames(bytes[3], bytes[2]);
  // vel_x = vel_x / 1000.0; //单位由：0.001m/s 转换为m/s

  double vel_x = static_cast<double>(Odom_Trans(bytes[2], bytes[3]));
  chassis_detail->mutable_gas()->set_throttle_output(vel_x);

  // double vel_y = parse_two_frames(bytes[5], bytes[4]);
  // vel_y = vel_y / 1000.0; //单位由：0.001m/s 转换为m/s
  
  double vel_y = static_cast<double>(Odom_Trans(bytes[4], bytes[5]));

  double vel_z = static_cast<double>(Odom_Trans(bytes[6], bytes[7]));
  // double vel_z = parse_two_frames(bytes[7], bytes[6]);
  // vel_z = vel_z / 1000.0;  //单位由：0.001rad/s 转换为rad/s
  chassis_detail->mutable_eps()->set_steering_angle_spd(vel_z);

  chassis_detail->mutable_robot_vel()->set_x(vel_x);
  chassis_detail->mutable_robot_vel()->set_y(vel_y);
  chassis_detail->mutable_robot_vel()->set_z(vel_z);
  chassis_detail->set_is_akman101(true);

  AWARN << " CurrentVel_X "<< vel_x;
  AWARN << " CurrentVel_Y "<< vel_y;
  AWARN << " CurrentVel_Z "<< vel_z;
}

double AkemanInfo101::pedal_input(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 2);
  // Pedal Input from the physical pedal
  return parse_two_frames(bytes[0], bytes[1]);
}

double AkemanInfo101::pedal_cmd(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 4);
  // Pedal Command from the command message
  return parse_two_frames(bytes[2], bytes[3]);
}

double AkemanInfo101::pedal_output(const std::uint8_t *bytes, int32_t length) const {
  DCHECK_GE(length, 6);
  // Pedal Output is the maximum of PI and PC
  return parse_two_frames(bytes[4], bytes[5]);
}

float AkemanInfo101::Odom_Trans(const std::uint8_t Data_High, const std::uint8_t Data_Low) const {

  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

double AkemanInfo101::parse_two_frames(const std::uint8_t low_byte,
                                 const std::uint8_t high_byte) const {
  Byte frame_high(&high_byte);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(&low_byte);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  // double output = 100.0 * value * 1.52590218966964e-05;
  // output = ProtocolData::BoundedValue(0.0, 100.0, output);
  // return output;

  if (value > 0x8000) {
    value = value - 0x10000;
  }
  value = value * 1.0;

  return value ;
}

bool AkemanInfo101::boo_input(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  // seems typo here
  return frame.is_bit_1(0);
}

bool AkemanInfo101::boo_cmd(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(1);
}

bool AkemanInfo101::boo_output(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  // seems typo here
  return frame.is_bit_1(2);
}

bool AkemanInfo101::is_watchdog_counter_applying_brakes(const std::uint8_t *bytes,
                                                  int32_t length) const {
  Byte frame(bytes + 6);
  return frame.is_bit_1(3);
}

int32_t AkemanInfo101::watchdog_counter_source(const std::uint8_t *bytes,
                                         int32_t length) const {
  // see table for status code
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(4, 4);
  return x;
}

bool AkemanInfo101::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

bool AkemanInfo101::is_driver_override(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(1);
}

bool AkemanInfo101::is_driver_activity(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool AkemanInfo101::is_watchdog_counter_fault(const std::uint8_t *bytes,
                                        int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool AkemanInfo101::is_channel_1_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(4);
}

bool AkemanInfo101::is_channel_2_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool AkemanInfo101::is_boo_switch_fault(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool AkemanInfo101::is_connector_fault(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo