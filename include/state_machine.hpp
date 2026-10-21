/**
 * @file state_machine.hpp
 * @author lupinjia (jason_1120202397@163.com)
 * @brief Simple state machine class for a robot.
 * @version 0.1
 * @date 2026-10-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "user_controller.hpp"

/**
 * @brief State enum for the robot state machine.
 * 
 * @details The robot machine can be in one of the following states:
 *          1. DAMPING: The robot is in a damping state, waiting for the fault to be recovered.
 *          2. STAND: The robot is in a standing state, ready to move.
 *          3. CTRL: The robot is in a control state, following the desired trajectory.
 *          4. TEST: The robot is in a test state, conducting some kind of test.
 *          5. ZERO_TORQUE: The robot is in a zero torque state, waiting for the fault to be recovered.
 */
enum class STATES
{
  DAMPING = 0,
  STAND = 1,
  CTRL = 2,
  TEST = 3,
  ZERO_TORQUE = 4
};

/**
 * @brief Simple state machine class for a robot.
 * 
 * @details This class is used to manage the state of the robot, and to transition between different states.
 */
class SimpleStateMachine
{
public:

  SimpleStateMachine() : state(STATES::ZERO_TORQUE) {
    state_name = {"DAMPING", "STAND", "CTRL", "TEST", "ZERO_TORQUE"};
  }

  /**
   * @brief Set state to damping.
   * @details If the robot is in a fault state, the transition will be rejected. 
   * @return true If the transition is successful.
   * @return false Otherwise.
   */
  bool damp()
  {
    if(fault_latched && !fault_reset_requested)
    {
      return false;
    }
    state = STATES::DAMPING;
    return true;
  }

  /**
   * @brief Set state to standing.
   * @details If the robot is in a fault state, the transition will be rejected. 
   * @return true If the transition is successful.
   * @return false Otherwise.
   */
  bool stand()
  {
    if(state == STATES::DAMPING)
    {
      state = STATES::STAND;
      standing_count = 0; // reset count
      return true;
    }
    else // only from DAMPING can one enter STAND state
    {
      return false;
    }
  }

  /**
   * @brief Set state to control.
   * @details If the robot is in a fault state, the transition will be rejected. 
   * @return true If the transition is successful.
   * @return false Otherwise.
   */
  bool ctrl()
  {
    if((state == STATES::DAMPING || state == STATES::STAND) &&
       (!fault_latched || fault_reset_requested))
    {
      state = STATES::CTRL;
      return true;
    }
    else
    {
      spdlog::warn("Cannot enter CTRL state from current state or before fault recovery!");
      return false;
    }
  }

  /**
   * @brief Set state to test.
   * @details If the robot is in a fault state, the transition will be rejected. 
   * @return true If the transition is successful.
   * @return false Otherwise.
   */
  bool test()
  {
    if(state == STATES::STAND || state == STATES::DAMPING)
    {
      state = STATES::TEST;
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief Set state to zero torque.
   * @details This is the fall-back state when robot is in a fault state.
   * @return true If the transition is successful.
   * @return false Otherwise.
   */
  bool zeroTorque()
  {
    state = STATES::ZERO_TORQUE;
    if(fault_latched)
    {
      fault_reset_requested = true;
    }
    return true;
  }

  /**
   * @brief Latch fault.
   * @details This method is used to set the fault flag to true, and transition the robot to the ZERO_TORQUE state.
   */
  void latchFault() noexcept
  {
    fault_latched = true;
    fault_reset_requested = false;
    state = STATES::ZERO_TORQUE;
  }

  /**
   * @brief Processing of standing state.
   * 
   * @param ctrl User controller object.
   */
  void standing(BasicUserController &ctrl)
  {
    // calc percent
    standing_percent = (float)standing_count / standing_duration;
    // calc desired pos
    for(uint8_t i = 0; i < 6; i++)
    {
      ctrl.jpos_des.at(i) = (1 - standing_percent) * ctrl.start_pos.at(i) + standing_percent * ctrl.stand_pos.at(i);
    }
    standing_count++;
    standing_count = standing_count > standing_duration? standing_duration : standing_count;
  }

  STATES state;
  std::vector<std::string> state_name;

protected:
  int standing_count = 0;       // start counting after entering STATES::STAND, for linear interpolation
  float standing_percent = 0.0; // percent of linear interpolation
  int standing_duration = 200;  // decides the period of standing
  bool fault_latched = false;
  bool fault_reset_requested = false;
};
#endif
