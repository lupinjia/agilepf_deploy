#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "user_controller.hpp"

enum class STATES
{
  DAMPING = 0,
  STAND = 1,
  CTRL = 2,
  TEST = 3
};

class SimpleStateMachine
{
public:

  SimpleStateMachine() : state(STATES::DAMPING) {}

  /**
   * @brief  stop control, set state to damping
   * 
   * @return true 
   * @return false 
   */
  bool stop()
  {
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
    if(state == STATES::STAND || state == STATES::DAMPING)
    {
      state = STATES::CTRL;
      return true;
    }
    else // only from STAND can one enter CTRL state
    {
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

protected:
  int standing_count = 0;       // start counting after entering STATES::STAND, for linear interpolation
  float standing_percent = 0.0; // percent of linear interpolation
  int standing_duration = 200;  // decides the period of standing
};
#endif