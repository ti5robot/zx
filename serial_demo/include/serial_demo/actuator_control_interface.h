#ifndef ACTUATOR_CONTROL_INTERFACE_H
#define ACTUATOR_CONTROL_INTERFACE_H
#define MAX_ACTUATOR 16
#include <stdint.h>
#include "uart_frame_communication_protocol.h"


#define SET_MOTOR_ENABLE 1
#define	SET_MOTOR_DISABLE	2
#define	GET_MOTOR_RUN_MODE 3
#define	GET_REG_I_Q	4
#define	GET_REG_I_Q_REF	5
#define	GET_REG_SPEED	6
#define	GET_REG_SPEED_REF	7
#define	GET_REG_CURRENT_POSITION	8
#define	GET_REG_TARGET_POSITION	9
#define	GET_REG_STATUS	10
#define	SET_FAULT_ACK	11
#define	CMD_ENCODER_ALIGN	12
#define	CMD_RESTORE_PARAMETER	13
#define	CMD_STORE_PARAMETER	14
#define	CMD_RECOVERY_FACTORY_SETTING	15
#define	GET_REG_SPEED_KP	16
#define	GET_REG_SPEED_KI	17
#define	GET_REG_POSITION_KP	18
#define	GET_REG_POSITION_KD	19
#define	GET_REG_BUS_VOLTAGE	20
#define	GET_REG_HEATS_TEMP	21
#define	GET_REG_MAX_APP_ACCEL	22
#define	GET_REG_MIN_APP_ACCEL	23
#define	GET_REG_MAX_APP_SPEED	24
#define	GET_REG_MIN_APP_SPEED	25
#define	GET_REG_MAX_APP_POSITION	26
#define	GET_REG_MIN_APP_POSITION	27

#define	SET_REG_TORQUE_REF	28
#define	SET_REG_RAMP_FINAL_SPEED	29
#define	SET_REG_TARGET_POSITION	30
#define	SET_REG_TARGET_POSITION_TRAPEZIUM	31
#define	SET_REG_MAX_APP_TORQUE	32
#define	SET_REG_MIN_APP_TORQUE	33
#define	SET_REG_MAX_APP_ACCEL	34
#define	SET_REG_MIN_APP_ACCEL	35
#define	SET_REG_MAX_APP_SPEED	36
#define	SET_REG_MIN_APP_SPEED	37
#define	SET_REG_MAX_APP_POSITION	38
#define	SET_REG_MIN_APP_POSITION	39
#define	SET_REG_SPEED_KP	41
#define	SET_REG_SPEED_KI	42
#define	SET_REG_POSITION_KP	43
#define	SET_REG_POSITION_KI	44
#define	SET_REG_POSITION_KD	45
#define	SET_REG_CAN_ID	46
#define SET_REG_I2T_PROTECTION	47
#define SET_OVER_TEMP_PROTECTION	48
#define	GET_REG_MOTOR_TEMP	49
#define	GET_REG_BOARD_TEMP	50
#define	GET_ACT_INFO	100			//执行器外径，减速比
#define	GET_CODE_VERSION	101
#define	GET_REG_CSP	65
#define	SET_TORQUE_GET_CSP	66
#define	SET_SPEED_GET_CSP	67
#define	SET_POSITION_GET_CSP	68
#define	SET_TARGET_POSITION_GET_CSP	69
#define	CMD_MANUFACTURE_1	127
#define	CMD_MANUFACTURE_2	128
#define	CMD_MANUFACTURE_3	129
#define IDLE 0
#define IDLE_START 1
#define FIND_ACT 2
#define CONTROL_ACT 3
#define SET_PARAMETER 4

struct Actuator_Control_State_Machine_t
{
  int32_t State;
  int32_t NextState;

};
typedef struct
{
  int index;
  int FoundCount;

}
Actuator_Find_t;
typedef struct
{

  uint32_t CanId;
  float TorqueRatio;
  int32_t RunMode;
  int32_t TargetIq_mA;
  int32_t Iq_mA;
  int32_t TargetSpeed;
  int32_t Speed;
  int32_t TargetPosition;
  int32_t Position;
  int32_t MaxIq_mA;
  int32_t MaxAcceleration;
  int32_t MinAcceleration;
  int32_t MaxSpeed;
  int32_t MinSpeed;
  int32_t MaxPosition;
  int32_t MinPosition;
  int32_t Speed_KpGain;
  int32_t Speed_KiGain;
  int32_t Position_KpGain;
  int32_t Position_KiGain;
  int32_t Position_KdGain;
  int32_t ErrStatus;
  int32_t MotorTemp;
  int32_t BoardTemp;
  int32_t BusVotage;
  uint32_t Infomation;
  uint32_t CodeVersion;
  uint32_t OnlineState;
  uint32_t Mode;
  uint32_t Zero;
  uint32_t ZeroRound;



}Acturator_t;
extern Acturator_t ActuatorList[MAX_ACTUATOR];
extern uint8_t IdToIndexMap[256];
class Actuator_Control_interface
{
public:
    Actuator_Control_State_Machine_t ACSM;
    Actuator_Find_t ActFind;













    Actuator_Control_interface();
};

#endif // ACTUATOR_CONTROL_INTERFACE_H
