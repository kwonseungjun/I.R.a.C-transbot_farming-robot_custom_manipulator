
#include "../../include/robotis_manipulator/robotis_manipulator_manager.h"

using namespace robotis_manipulator;

bool JointActuator::findId(uint8_t actuator_id)
{
  std::vector<uint8_t> id = getId();
  for(uint32_t index = 0; index < id.size(); index++)
  {
    if(id.at(index) == actuator_id)
      return true;
  }
  return false;
}

bool JointActuator::getEnabledState()
{
  return enabled_state_;
}

bool ToolActuator::findId(uint8_t actuator_id)
{
  if(getId() == actuator_id)
  {
    return true;
  }
  return false;
}

bool ToolActuator::getEnabledState()
{
  return enabled_state_;
}
