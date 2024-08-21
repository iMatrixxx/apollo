#pragma once

#include "modules/canbus_vehicle/lincoln/proto/lincoln.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

/**
 * @class Accel103
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Accel103 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Lincoln> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     Lincoln *chassis_detail) const;

 private:

  double parse_two_frames(const std::uint8_t low_byte,
                          const std::uint8_t high_byte) const;
  short IMU_Trans(const std::uint8_t Data_High, const std::uint8_t Data_Low) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
