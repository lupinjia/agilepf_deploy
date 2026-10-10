#ifndef __ROBOT_INTERFACE_HPP__
#define __ROBOT_INTERFACE_HPP__

#include <array>
#include <chrono>
#include <cmath>
#include <legged_sdk/idl/agilepf/LowState/LowState.hpp>
#include <common/mathTools.h>

class BasicRobotInterface
{
public:

  BasicRobotInterface()
  {
    jpos_des.fill(0.);
    jvel_des.fill(0.);
    jpos.fill(0.);
    jvel.fill(0.);
    kp.fill(0.);
    kd.fill(4.);
    tau_ff.fill(0.);
    projected_gravity.fill(0.);
    projected_gravity.at(2) = -1.0;
    rpy_offset.fill(0.);
  }

  void updateLowState(legged::idl::agilepf::LowState &low_state)
  {
    // imu
    const legged::idl::agilepf::IMUState &imu = low_state.imu_state();
    std::array<float, 4> raw_quat = imu.quaternion();
    // apply rpy offset
    auto quat_offset = rpyToQuat(rpy_offset);
    quat = quatMultiply(quat_offset, raw_quat);
    rpy = imu.rpy();
    gyro = imu.gyroscope();
    acc = imu.accelerometer();
    updateProjectedGravity();
    
    // motor
    const std::array<legged::idl::agilepf::MotorState, 6> &motor = low_state.motor_state();
    for (size_t i = 0; i < 6; ++i)
    {
      const legged::idl::agilepf::MotorState &m = motor.at(i);
      jpos.at(i) = m.q();
      jvel.at(i) = m.dq();
    }
  }

  // robot state
  std::array<float, 6> jpos, jvel;
  std::array<float, 4> quat;
  std::array<float, 3> rpy, gyro, projected_gravity, acc;
  // robot cmd
  std::array<float, 6> jpos_des, jvel_des, kp, kd, tau_ff;
  // imu offset
  std::array<float, 3> rpy_offset;

private:
  inline void updateProjectedGravity()
  {
    // inverse quat
    float w = quat.at(0);
    float x = -quat.at(1);
    float y = -quat.at(2);
    float z = -quat.at(3);

    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;
    float w2 = w * w;

    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;


    projected_gravity.at(0) = -2 * (xz + wy);
    projected_gravity.at(1) = -2 * (yz - wx);
    projected_gravity.at(2) = -(w2 - x2 - y2 + z2);
  }
};

#endif
