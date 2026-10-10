#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "user_controller.hpp"

enum class STATES
{
  DAMPING = 0,
  STAND = 1,
  CTRL = 2,
  TEST = 3,
  ZERO_TORQUE = 4
};

class SimpleStateMachine
{
public:

  SimpleStateMachine() : state(STATES::ZERO_TORQUE) {
    state_name = {"DAMPING", "STAND", "CTRL", "TEST", "ZERO_TORQUE"};
  }

  /**
   * @brief  set state to damping
   * 
   * @return true 
   * @return false 
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

  bool zeroTorque()
  {
    state = STATES::ZERO_TORQUE;
    if(fault_latched)
    {
      fault_reset_requested = true;
    }
    return true;
  }

  void latchFault() noexcept
  {
    fault_latched = true;
    fault_reset_requested = false;
    state = STATES::ZERO_TORQUE;
  }

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
