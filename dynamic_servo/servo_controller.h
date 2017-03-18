#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 
//
#include <string>

using namespace std; 

#define SWITCH 1		// This is corresponding to GPIO 18. In wiringPi it is 1. 
#define SWITCH_ON 1	 	// SWITCH_ON = TX enable, RPi in transmitting mode. 
#define SWITCH_OFF 0   	// SWITCH_OFF = RX enable, RPi in receiving mode. 

//important AX-12 constants
/////////////////////////////////////////////////////////// EEPROM AREA
#define    AX_MODEL_NUMBER_L 0x00
#define    AX_MODEL_NUMBER_H 0x01
#define    AX_VERSION 0x02
#define    AX_ID 0x03
#define    AX_BAUD_RATE 0x04
#define    AX_RETURN_DELAY_TIME 0x05
#define    AX_CW_ANGLE_LIMIT_L 0x06
#define    AX_CW_ANGLE_LIMIT_H 0x07
#define    AX_CCW_ANGLE_LIMIT_L 0x08
#define    AX_CCW_ANGLE_LIMIT_H 0x09
#define    AX_SYSTEM_DATA2 0x0A  //????
#define    AX_HIGHEST_LIMIT_TEMPERATURE 0x0B
#define    AX_LOWEST_LIMIT_VOLTAGE 0x0C
#define    AX_HIGHEST_LIMIT_VOLTAGE 0x0D
#define    AX_MAX_TORQUE_L 0x0E
#define    AX_MAX_TORQUE_H 0x0F
#define    AX_RETURN_LEVEL 0x10
#define    AX_ALARM_LED 0x11
#define    AX_ALARM_SHUTDOWN 0x12
#define    AX_OPERATING_MODE 0x13
#define    AX_DOWN_CALIBRATION_L 0x14
#define    AX_DOWN_CALIBRATION_H 0x15
#define    AX_UP_CALIBRATION_L 0x16
#define    AX_UP_CALIBRATION_H 0x17


////////////////////////////////////////////////////////////// RAM AREA
#define    AX_TORQUE_ENABLE 0x18
#define    AX_LED_ENABLE 0x19
#define    AX_CW_COMPLIANCE_MARGIN 0x1A
#define    AX_CCW_COMPLIANCE_MARGIN 0x1B
#define    AX_CW_COMPLIANCE_SLOPE 0x1C
#define    AX_CCW_COMPLIANCE_SLOPE 0x1D
#define    AX_GOAL_POSITION_L 0x1E
#define    AX_GOAL_POSITION_H 0x1F
#define    AX_MOVE_SPEED_L 0x20
#define    AX_MOVE_SPEED_H 0x21
#define    AX_TORQUE_LIMIT_L 0x22
#define    AX_TORQUE_LIMIT_H 0x23
#define    AX_PRESENT_POSITION_L 0x24
#define    AX_PRESENT_POSITION_H 0x25
#define    AX_PRESENT_SPEED_L 0x26
#define    AX_PRESENT_SPEED_H 0x27
#define    AX_PRESENT_LOAD_L 0x28
#define    AX_PRESENT_LOAD_H 0x29
#define    AX_PRESENT_VOLTAGE 0x2A
#define    AX_PRESENT_TEMPERATURE 0x2B
#define    AX_REGISTERED_INSTRUCTION 0x2C
#define    AX_PAUSE_TIME 0x2D
#define    AX_MOVING 0x2E
#define    AX_LOCK 0x2F
#define    AX_PUNCH_L 0x30
#define    AX_PUNCH_H 0x31

/////////////////////////////////////////////////////////////// Status Return Levels
#define    AX_RETURN_NONE 0x0
#define    AX_RETURN_READ 0x1
#define    AX_RETURN_ALL 0x2

/////////////////////////////////////////////////////////////// Instruction Set
#define    AX_PING 0x1
#define    AX_READ_DATA 0x2
#define    AX_WRITE_DATA 0x3
#define    AX_REG_WRITE 0x4
#define    AX_ACTION 0x5
#define    AX_RESET 0x6
#define    AX_SYNC_WRITE 0x83

/////////////////////////////////////////////////////////////// Lengths
#define    AX_RESET_LENGTH 2
#define    AX_ACTION_LENGTH 2
#define    AX_ID_LENGTH 4
#define    AX_LR_LENGTH 4
#define    AX_SRL_LENGTH 4
#define    AX_RDT_LENGTH 4
#define    AX_LEDALARM_LENGTH 4
#define    AX_SHUTDOWNALARM_LENGTH 4
#define    AX_TL_LENGTH 4
#define    AX_VL_LENGTH 6
#define    AX_AL_LENGTH 7
#define    AX_CM_LENGTH 6
#define    AX_CS_LENGTH 5
#define    AX_COMPLIANCE_LENGTH 7
#define    AX_CCW_CW_LENGTH 8
#define    AX_BD_LENGTH 4
#define    AX_TEM_LENGTH 4
#define    AX_MOVING_LENGTH 4
#define    AX_RWS_LENGTH 4
#define    AX_VOLT_LENGTH 4
#define    AX_LOAD_LENGTH 4
#define    AX_LED_LENGTH 4
#define    AX_TORQUE_LENGTH 4
#define    AX_POS_LENGTH 4
#define    AX_GOAL_LENGTH 5
#define    AX_MT_LENGTH 5
#define    AX_PUNCH_LENGTH 5
#define    AX_SPEED_LENGTH 5
#define    AX_GOAL_SP_LENGTH 7

/////////////////////////////////////////////////////////////// Specials
#define    AX_BYTE_READ 0x1
#define    AX_INT_READ 0x2
#define    AX_ACTION_CHECKSUM 0xFA
#define    AX_BROADCAST_ID 0xFE
#define    AX_START 0xFF
#define    AX_CCW_AL_L 0xFF
#define    AX_CCW_AL_H 0x3
#define    AX_LOCK_VALUE 0x1

//-----------------------------------Function-----------------------------------
void printData(char data[], int len);

int writeData(int fd, char *data, int dataLen);

int readData(int fd, char* buffer, int len);

int readRegister(int fd, int id, int inst);

void torqueEnable(int fd, int id, int enable);

void setToWheelMode(int fd, int id);

void setToJointMode(int fd, int id, int cwLimit, int ccwLimit);

void setPosition(int fd, int id, int position);

void setTorqueLimit(int fd, int id, int torque);

void setMovingSpeed(int fd, int id, int speed);

void setMaxTorque(int fd, int id, int torque);

int readPresentPosition(int fd, int id);

void servoControl(int fd, int id, int CW_limit, int CCW_limit, int step_len, int speed);

void setReturnDelayTime(int fd, int id, int dalay_time);
