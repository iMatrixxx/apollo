
#include "modules/canbus/tools/Quaternion_Solution.h"

namespace apollo {
namespace canbus {

AkmanQuaternionSolution::AkmanQuaternionSolution() {
    twoKp = 1.0;     // 2 * proportional gain (Kp)
    twoKi = 0.0;     // 2 * integral gain (Ki)
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
    integralFBx = 0.0;
    integralFBy = 0.0;
    integralFBz = 0.0; // integral error terms scaled by Ki
}
/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到
***************************************/
float AkmanQuaternionSolution::InvSqrt(float number)
{
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5f;
  // if (number <= 0.0f) {
  //   return 0.0f;
  // }
  x = number * 0.5;
  y = number;
  i = * (( long * ) &y);
  i = 0x5f375a86 - ( i >> 1 );
  y = * (( float * ) &i);
  y = y * ( f - ( x * y * y ) );

  return y;
}
/**************************************
Date: May 31, 2020
Function: 四元数解算
***************************************/

void AkmanQuaternionSolution::Quaternion_Solution(double gx, double gy, double gz, 
  double ax, double ay, double az, apollo::common::Quaternion& Mpu6050) {
  
  // AWARN << "debug1 --------------------------------------";
  // AWARN << "q0:" << q0 << "  q1:" << q1 << "  q2:" << q2 << "  q3:" << q3;
  
  double recipNorm = 0.0;
  double halfvx = 0.0, halfvy = 0.0, halfvz = 0.0;
  double halfex =0.0, halfey = 0.0, halfez = 0.0;
  double qa = 0.0, qb = 0.0, qc = 0.0;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    // AWARN << "debug 2 --------------------------------------";
    // AWARN << "ax:" << ax << "  ay:" << ay << "  az:" << az;
    double sqrt_num = std::pow(ax, 2) + std::pow(ay, 2) + std::pow(az, 2);
    recipNorm = InvSqrt(sqrt_num);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0 / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0 / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0 / SAMPLING_FREQ);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0;       // prevent integral windup
      integralFBy = 0.0;
      integralFBz = 0.0;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5 * (1.0 / SAMPLING_FREQ));   // pre-multiply common factors
  gy *= (0.5 * (1.0 / SAMPLING_FREQ));
  gz *= (0.5 * (1.0 / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  // Normalise quaternion
  // AWARN << "debug 3 --------------------------------------";
  // AWARN << "q0:" << q0 << "  q1:" << q1 << "  q2:" << q2 << "  q3:" << q3;
  double sqrt_num = std::pow(q0, 2) + std::pow(q1, 2) + std::pow(q2, 2) + std::pow(q3, 2);
  //AWARN << "sqrt_num q:" << sqrt_num;

  recipNorm = InvSqrt(sqrt_num);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // AWARN << "debug 4 --------------------------------------";
  // AWARN << "q0:" << q0 << "  q1:" << q1 << "  q2:" << q2 << "  q3:" << q3;
  // AWARN << "recipNorm:" << recipNorm;

  Mpu6050.set_qw(q0);
  Mpu6050.set_qx(q1);
  Mpu6050.set_qy(q2);
  Mpu6050.set_qz(q3);

  return ;
}

}
}
