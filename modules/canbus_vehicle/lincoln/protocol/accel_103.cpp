#include "modules/canbus_vehicle/lincoln/protocol/accel_103.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Accel103::ID = 0x103; //zhxf 20240725 阿克曼小车

void Accel103::Parse(const std::uint8_t *bytes, int32_t length,
                    Lincoln *chassis_detail) const {


  double angle_vel_y = parse_two_frames(bytes[1], bytes[0]) / 3753.0;
  double angle_vel_z = parse_two_frames(bytes[3], bytes[2]) / 3753.0;
  double battery_voltage = parse_two_frames(bytes[5], bytes[4]) / 1000.0;

  AWARN << "Angle_Vel_Y "<<angle_vel_y;
  AWARN << "Angle_Vel_Z "<<angle_vel_z;
  AWARN << "Battery_Voltage(V) "<<battery_voltage;

}

double Accel103::parse_two_frames(const std::uint8_t low_byte,
                                 const std::uint8_t high_byte) const {
  Byte high_frame(&high_byte);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(&low_byte);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x7FFF) {
    value -= 0x10000;
  }
  return value; //zhxf 20240725 阿克曼小车
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo