#include "actuator_control_interface.h"
Acturator_t ActuatorList[MAX_ACTUATOR];
uint8_t IdToIndexMap[256];
Actuator_Control_interface::Actuator_Control_interface()
{
    this->ACSM.State = 0;
    this->ACSM.NextState = 0;
    memset(ActuatorList,0,sizeof(ActuatorList[MAX_ACTUATOR]));
}
