
// #ifndef __QUATERNION_SOLUTION_H_
// #define __QUATERNION_SOLUTION_H_
#pragma once
#include <iostream>
#include "cyber/cyber.h"
#include "modules/common_msgs/basic_msgs/geometry.pb.h"

namespace apollo {
namespace canbus {
#define SAMPLING_FREQ 100.0f // 采样频率

class AkmanQuaternionSolution
{
public:
    AkmanQuaternionSolution();
    ~AkmanQuaternionSolution() = default;
    void Quaternion_Solution(double gx, double gy, 
                             double gz, double ax, 
                             double ay, double az, 
                             apollo::common::Quaternion& Mpu6050);
    float InvSqrt(float number);

    
private:
    double twoKp = 1.0;     // 2 * proportional gain (Kp)
    double twoKi = 0.0;     // 2 * integral gain (Ki)
    double q0 = 1.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
    double integralFBx = 0.0;
    double integralFBy = 0.0;
    double integralFBz = 0.0; // integral error terms scaled by Ki

};
}
}


// #endif


