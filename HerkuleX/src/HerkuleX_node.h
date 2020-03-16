#ifndef HERKULEX_PROTOCOL_H_
#define HERKULEX_PROTOCOL_H_

#define API 
#define CALLING_CONVENTION

//////////////////////////////프로토콜에 대한 선언//////////////////////////////
// status error
#define EXCEED_INPUT_VOLTAGE_LIMIT	0x01
#define EXCEED_ALLOWED_POT_LIMIT	0x02
#define EXCEED_TEMPERATURE_LIMIT	0x04
#define INVALID_PACKET				0x08
#define OVERLOAD_DETECTED			0x10
#define DRIVER_FAULT_DETECTED		0x20
#define EEP_REG_DISORTED			0x40

// status detail
#define MOVING_FLAG					0x01
#define INPOSITION_FLAG				0x02
#define CHEKSUM_ERROR				0x04
#define UNKNOWN_COMMAND				0x08
#define EXCEED_REG_RANGE			0x10
#define GARBAGE_DETECTED			0x20
#define MOTOR_ON_FLAG				0x40

// LED COLOR
#define LED_GREEN					0x01
#define LED_BLUE					0x02
#define LED_RED						0x04

// TORQUE_CONTROL
#define TORQUE_CONTROL_FREE			0x00
#define TORQUE_CONTROL_BRAKEON		0x40
#define TORQUE_CONTROL_TORQUEON		0x60

enum EEP_REGISTER
{
	EEP_MODEL_NO_1 = 0,
	EEP_MODEL_NO_2 = 1,
	EEP_VERSION_1 = 2,
	EEP_VERSION_2 = 3,
	EEP_BAUD_RATE = 4,
	EEP_REASERVED_1 = 5,
	EEP_ID = 6,
	EEP_ACK_POLICY = 7,
	EEP_ALARM_LED_POLICY = 8,
	EEP_TORQUE_POLICY = 9,
	EEP_RESERVED_2 = 10,
	EEP_MAX_TEMPERATURE = 11,
	EEP_MIN_VOLTAGE = 12,
	EEP_MAX_VOLTAGE = 13,
	EEP_ACCELERATION_RATIO = 14,
	EEP_MAX_ACCELERATION_TIME = 15,
	EEP_DEAD_ZONE = 16,
	EEP_SATURATOR_OFFSET = 17,
	EEP_SATURATOR_SLOPE = 18,
	EEP_PWM_OFFSET = 20,
	EEP_MIN_PWM = 21,
	EEP_MAX_PWM = 22,
	EEP_OVERLOAD_PWM_THRESHOLD = 24,
	EEP_MIN_POSITION = 26,
	EEP_MAX_POSITION = 28,
	EEP_POSITION_KP = 30,
	EEP_POSITION_KD = 32,
	EEP_POSITION_KI = 34,
	EEP_POSITION_FEEDFORWARD_1ST_GAIN = 36,
	EEP_POSITION_FEEDFORWARD_2ND_GAIN = 38,
	EEP_RESERVED_3 = 40,
	EEP_RESERVED_4 = 42,
	EEP_VELOCITY_KP = 38,
	EEP_VELOCITY_KD = 40,
	EEP_VELOCITY_KI = 42,
	EEP_LED_BLINK_PERIOD = 44,
	EEP_ADC_FAULT_CHECK_PERIOD = 45,
	EEP_PACKET_GARBAGE_CHECK_PERIOD = 46,
	EEP_STOP_DETECTION_PERIOD = 47,
	EEP_OVELOAD_DETECTION_PERIOD = 48,
	EEP_STOP_THRESHOLD = 49,
	EEP_INPOSITION_MARGIN = 50,
	EEP_RESERVED_5 = 51,
	EEP_RESERVED_6 = 52,
	EEP_CALIBRATION_DIFFERENCE = 53,
	EEP_LAST = EEP_CALIBRATION_DIFFERENCE
};

enum RAM_REGISTER
{
	RAM_ID = 0,
	RAM_ACK_POLICY = 1,
	RAM_ALARM_LED_POLICY = 2,
	RAM_TORQUE_POLICY = 3,
	RAM_RESERVED_2 = 4,
	RAM_MAX_TEMPERATURE = 5,
	RAM_MIN_VOLTAGE = 6,
	RAM_MAX_VOLTAGE = 7,
	RAM_ACCELERATION_RATIO = 8,
	RAM_MAX_ACCELERATION_TIME = 9,
	RAM_DEAD_ZONE = 10,
	RAM_SATURATOR_OFFSET = 11,
	RAM_SATURATOR_SLOPE = 12,
	RAM_PWM_OFFSET = 14,
	RAM_MIN_PWM = 15,
	RAM_MAX_PWM = 16,
	RAM_OVERLOAD_PWM_THRESHOLD = 18,
	RAM_MIN_POSITION = 20,
	RAM_MAX_POSITION = 22,
	RAM_POSITION_KP = 24,
	RAM_POSITION_KD = 26,
	RAM_POSITION_KI = 28,
	RAM_POSITION_FEEDFORWARD_1ST_GAIN = 30,
	RAM_POSITION_FEEDFORWARD_2ND_GAIN = 32,
	RAM_RESERVED_3 = 34,
	RAM_RESERVED_4 = 36,
	RAM_VELOCITY_KP = 32,
	RAM_VELOCITY_KD = 34,
	RAM_VELOCITY_KI = 36,
	RAM_LED_BLINK_PERIOD = 38,
	RAM_ADC_FAULT_CHECK_PERIOD = 39,
	RAM_PACKET_GARBAGE_CHECK_PERIOD = 40,
	RAM_STOP_DETECTION_PERIOD = 41,
	RAM_OVELOAD_DETECTION_PERIOD = 42,
	RAM_STOP_THRESHOLD = 43,
	RAM_INPOSITION_MARGIN = 44,
	RAM_TURN = 45, //RAM_RESERVED_5
	//RAM_CALIBRATION_OFFSET = 46,	// DRS-0102, DRS-0202 only
	RAM_CALIBRATION__DIFFERENCE_L = 46,
	RAM_CALIBRATION__DIFFERENCE_H = 47,
	RAM_STATUS_ERROR = 48,
	RAM_STATUS_DETAIL = 49,
	RAM_RESERVED_7 = 50,
	RAM_RESERVED_8 = 51,
	RAM_TORQUE_CONTROL = 52,
	RAM_LED_CONTROL = 53,
	RAM_VOLTAGE = 54,
	RAM_TEMPERATURE = 55,
	RAM_CURRENT_CONTROL_MODE = 56,
	RAM_TICK = 57,
	RAM_CALIBRATED_POSITION = 58,
	RAM_ABSOLUTE_POSITION = 60,
	RAM_DIFFERENTIAL_POSITION = 62,
	RAM_PWM = 64,
	RAM_RESERVED_9 = 66,
	RAM_ABSOLUTE_GOAL_POSITION = 68,
	RAM_ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70,
	RAM_DESIRED_VELOCITY = 72,
	RAM_LAST = RAM_DESIRED_VELOCITY
};

typedef struct IJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;

	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char PlayTime_ms;

	unsigned char Profile; //S��: 1, ��ٸ���: 0 ����
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;

	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;

}IJog;

typedef struct SJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;

	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char Padding0;

	unsigned char Profile; //S��: 1, ��ٸ���: 0 ����
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;

	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;

}SJog;

typedef struct MIJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;

	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char PlayTime_ms;

	unsigned char Profile; //S��: 1, ��ٸ���: 0 ����
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;

	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;

}MIJog;

typedef struct MSJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;

	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char Padding0;

	unsigned char Profile; //S��: 1, ��ٸ���: 0 ����
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;

	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;

}MSJog;

typedef struct EEPRegisterMap
{
	unsigned char ucModelNo1;
	unsigned char ucModelNo2;
	unsigned char ucVersion1;
	unsigned char ucVersion2;
	unsigned char ucBaudRate;
	unsigned char ucReaserved1;
	unsigned char ucID;
	unsigned char ucAckPolicy;
	unsigned char ucAlarmLEDPolicy;
	unsigned char ucTorquePolicy;
	unsigned char ucReserved2;
	unsigned char ucMaxTemperature;
	unsigned char ucMinVoltage;
	unsigned char ucMaxVoltage;
	unsigned char ucAccelerationRatio;
	unsigned char ucMaxAccelerationTime;
	unsigned char ucDeadZone;
	unsigned char ucSaturatorOffset;
	unsigned short usSaturatorSlope;
	char cPWMOffset;
	unsigned char ucMinPWM;
	unsigned short usMaxPWM;
	unsigned short usOverloadPWMThreshold;
	unsigned short usMinPosition;
	unsigned short usMaxPosition;
	unsigned short usPositionKp;
	unsigned short usPositionKd;
	unsigned short usPositionKi;
	unsigned short usPositionFeedforward1stGain;
	unsigned short usPositionFeedforward2ndGain; //->usVelocityKp�� ���//
	//unsigned short usReserved3;
	//unsigned short usReserved4;
	unsigned short usVelocityKd;
	unsigned short usVelocityKi;
	unsigned char ucLEDBlinkPeriod;
	unsigned char ucADCFaultCheckPeriod;
	unsigned char ucPacketGarbageCheckPeriod;
	unsigned char ucStopDetectionPeriod;
	unsigned char ucOverloadDetectionPeriod;
	unsigned char ucStopThreshold;
	unsigned char ucInpositionMargin;
	unsigned char ucReserved5;
	//unsigned char ucReserved6;
	//unsigned short usCalibrationOffet;
	char cCalibrationDifference_L;
	char cCalibrationDifference_H;
}EEPRegisterMap;

typedef struct  RAMRegisterMap
{
	unsigned char ucID;
	unsigned char ucAckPolicy;
	unsigned char ucAlarmLEDPolicy;
	unsigned char ucTorquePolicy;
	unsigned char ucReserved2;
	unsigned char ucMaxTemperature;
	unsigned char ucMinVoltage;
	unsigned char ucMaxVoltage;
	unsigned char ucAccelerationRatio;
	unsigned char ucMaxAccelerationTime;
	unsigned char ucDeadZone;
	unsigned char ucSaturatorOffset;
	unsigned short usSaturatorSlope;
	char cPWMOffset;
	unsigned char ucMinPWM;
	unsigned short usMaxPWM;
	unsigned short usOverloadPWMThreshold;
	unsigned short usMinPosition;
	unsigned short usMaxPosition;
	unsigned short usPositionKp;
	unsigned short usPositionKd;
	unsigned short usPositionKi;
	unsigned short usPositionFeedforward1stGain;
	unsigned short usPositionFeedforward2ndGain;
	//unsigned short usReserved3;
	//unsigned short usReserved4;
	unsigned short usVelocityKd;
	unsigned short usVelocityKi;
	unsigned char ucLEDBlinkPeriod;
	unsigned char ucADCFaultCheckPeriod;
	unsigned char ucPacketGarbageCheckPeriod;
	unsigned char ucStopDetectionPeriod;
	unsigned char ucOverloadDetectionPeriod;
	unsigned char ucStopThreshold;
	unsigned char ucInpositionMargin;
	unsigned char ucTurn; //Only New HerkuleX
	//unsigned char usCalibrationOffset;
	//unsigned char ucCalibrationDifference;
	char cCalibrationDifference_L;
	char cCalibrationDifference_H;
	unsigned char ucStatusError;
	unsigned char ucStatusDetail;
	unsigned char ucReserved7;
	unsigned char ucReserved8;
	unsigned char ucTorqueControl;
	unsigned char ucLEDControl;
	unsigned char ucVoltage;
	unsigned char ucTemperature;
	unsigned char ucCurrentControlMode;
	unsigned char ucTick;
	unsigned short usCalibratedPosition;
	unsigned short usAbsolutePosition;
	short sDifferentialPosition;
	unsigned short usPWM;
	unsigned short usReserved9;
	unsigned short usAbsoluteGoalPosition;
	unsigned short usAbsoluteDesiredTrajectoryPosition;
	short sDesiredVelocity;
} RAMRegisterMap;

//////////////////////////////프로토콜에 대한 선언//////////////////////////////
//각 항목의 인덱스
#define PROTOCOL_SIZE_IDX				2
#define PROTOCOL_ID_IDX					3
#define PROTOCOL_CMD_IDX				4
#define PROTOCOL_CS1_IDX				5
#define PROTOCOL_CS2_IDX				6
#define PROTOCOL_DATA_IDX				7

//헤더 관련
#define HEADER							0xFFFF

//SIZE 관련
#define MIN_PACKET_SIZE					7
#define MIN_ACK_PACKET_SIZE				9
#define MAX_PACKET_SIZE					223
#define MAX_DATA_SIZE					(MAX_PACKET_SIZE-MIN_PACKET_SIZE)

//ID 관련
#define MAX_ID							0xFD    
#define BROADCAST_ID					0xFE  

//CMD 관련 - Request Packet
#define CMD_EEP_WRITE					0x01
#define CMD_EEP_READ					0x02
#define CMD_RAM_WRITE					0x03
#define CMD_RAM_READ					0x04    
#define CMD_RW_DATA_ADDR_IDX			7
#define CMD_RW_DATA_LEN_IDX				8
#define CMD_I_JOG						0x05
#define CMD_I_JOG_STRUCT_SIZE			5
#define CMD_I_JOG_MAX_DRS				(MAX_DATA_SIZE/CMD_I_JOG_STRUCT_SIZE)
#define CMD_MI_JOG						0x0A
#define CMD_MI_JOG_STRUCT_SIZE			5
#define CMD_MI_JOG_MAX_DRS				(MAX_DATA_SIZE/CMD_MI_JOG_STRUCT_SIZE)
#define CMD_S_JOG						0x06
#define CMD_S_JOG_STRUCT_SIZE			4
#define CMD_S_JOG_MAX_DRS				(MAX_DATA_SIZE/CMD_S_JOG_STRUCT_SIZE)
#define CMD_MS_JOG						0x0B
#define CMD_MS_JOG_STRUCT_SIZE			4
#define CMD_MS_JOG_MAX_DRS				(MAX_DATA_SIZE/CMD_MS_JOG_STRUCT_SIZE)

//Add _원점셋팅 명령추가//
#define CMD_CALSET						0x0C

#define CMD_STAT						0x07    
#define CMD_ROLLBACK					0x08
#define CMD_REBOOT						0x09

#define CMD_MIN							(CMD_EEP_WRITE)
#define CMD_MAX							(CMD_REBOOT)

//CMD 관련 - ACK Packet
#define CMD_ACK_MASK					0x40

#define CMD_EEP_WRITE_ACK				(CMD_EEP_WRITE|CMD_ACK_MASK)
#define CMD_EEP_READ_ACK				(CMD_EEP_READ|CMD_ACK_MASK)
#define CMD_RAM_WRITE_ACK				(CMD_RAM_WRITE|CMD_ACK_MASK)
#define CMD_RAM_READ_ACK				(CMD_RAM_READ|CMD_ACK_MASK)
#define CMD_I_JOG_ACK					(CMD_I_JOG|CMD_ACK_MASK)
#define CMD_S_JOG_ACK					(CMD_S_JOG|CMD_ACK_MASK)
#define CMD_MI_JOG_ACK					(CMD_MI_JOG|CMD_ACK_MASK)
#define CMD_MS_JOG_ACK					(CMD_MS_JOG|CMD_ACK_MASK)
#define CMD_STAT_ACK					(CMD_STAT|CMD_ACK_MASK)
#define CMD_ROLLBACK_ACK				(CMD_ROLLBACK|CMD_ACK_MASK)
#define CMD_REBOOT_ACK					(CMD_REBOOT|CMD_ACK_MASK)

#define CMD_ACK_MIN						(CMD_EEP_WRITE_ACK)
#define CMD_ACK_MAX						(CMD_REBOOT_ACK)

//CheckSum 관련
#define CHKSUM_MASK							0xFE

#pragma pack(push,1)

//////////////////////////////프로토콜 구조체//////////////////////////////
typedef struct DrsJogPosition
{
	short	iValue : 16;
	//unsigned short	uiValue : 15;
	//unsigned short	reserved : 1;
}DrsJogPosition;

typedef struct DrsJogInfinite
{
	unsigned short	uiValue : 14;
	unsigned short	Direction : 1;
	unsigned short	reserved : 1;
}DrsJogInfinite;

typedef union DrsJog
{
	DrsJogPosition Position;
	DrsJogInfinite Infinite;
}DrsJog;

//Add///////////////////////////////////////////////////////
typedef struct DrsTurnPosition
{
	//unsigned short	uiValue;
	unsigned char	uiValue;
}DrsTurnPosition;

typedef union DrsMJog
{
	DrsJogPosition Position;
	DrsJogInfinite Infinite;
	//DrsTurnPosition Turn;
}DrsMJog;

typedef union DrsMJog2
{
	DrsTurnPosition Turn;
}DrsMJog2;
///////////////////////////////////////////////////////////////

typedef struct DrsSet
{
	unsigned char	ucStopFlag : 1;
	unsigned char	ucMode : 1;
	unsigned char	ucLedGreen : 1;
	unsigned char	ucLedBlue : 1;
	unsigned char	ucLedRed : 1;
	unsigned char	ucJogInvalid : 1;
	unsigned char	reserved : 1;
	unsigned char	ucProfile : 1;	// 0 : Trapezoidal, 1 : S-Curved
}DrsSet;

typedef struct DrsIJog
{
	DrsJog			stJog;
	DrsSet			stSet;
	unsigned char	ucId;
	unsigned char	ucPlayTime;
}DrsIJog;

typedef struct DrsSJog
{
	DrsJog			stJog;
	DrsSet			stSet;
	unsigned char	ucId;
}DrsSJog;

typedef struct DrsMIJog
{
	DrsMJog			stJog;
	DrsMJog2	    stJog2;
	DrsSet			stSet;
	unsigned char	ucId;
	unsigned char	ucPlayTime;
}DrsMIJog;

typedef struct DrsMSJog
{
	DrsMJog			stJog;
	DrsMJog2	    stJog2;
	DrsSet			stSet;
	unsigned char	ucId;
}DrsMSJog;

typedef struct DrsSJogData
{
	unsigned char	ucPlayTime;
	DrsSJog			stSJog[];
}DrsSJogData;

typedef struct DrsMSJogData
{
	unsigned char	ucPlayTime;
	DrsMSJog		stMSJog[];
}DrsMSJogData;

typedef struct DrsRWData
{
	unsigned char	ucAddress;
	unsigned char	ucLen;
	unsigned char	ucData[];
}DrsRWData;

typedef struct DrsStatData
{
	unsigned char ucError;
	unsigned char ucDetail;
}DrsStatData;


typedef struct DrsRollbackData
{
	unsigned char ucIdSkip;
	unsigned char ucBaudSkip;
}DrsRollbackData;

typedef union DrsData
{
	unsigned char	ucData[MAX_PACKET_SIZE - MIN_PACKET_SIZE];

	DrsRWData			stRWData;
	DrsIJog				stIJog[CMD_I_JOG_MAX_DRS];
	DrsSJogData			stSJogData;
	DrsMIJog			stMIJog[CMD_MI_JOG_MAX_DRS];
	DrsMSJogData		stMSJogData;
	DrsStatData			stStatData;
	DrsRollbackData		stRollbackData;
}DrsData;

typedef struct
{
	unsigned short			ucHeader;
	unsigned char			ucPacketSize;
	unsigned char			ucChipID;
	unsigned char			ucCmd;
	unsigned char			ucCheckSum1;
	unsigned char			ucCheckSum2;
	DrsData					unData;
}DrsPacket;

/************************************************************************************************************************************************************/
// S_JOG Move Struct Define
typedef struct CMD_SJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;
	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char Padding0;
	unsigned char Profile; 
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;
	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;
}CMD_SJog;

// I_JOG Move Struct Define
typedef struct CMD_IJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;
	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char PlayTime_ms;
	unsigned char Profile; 
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;
	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;
}CMD_IJog;

// MS_JOG Move Struct Define -> Only DRS-03xx series
typedef struct CMD_MSJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;
	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char Padding0;
	unsigned char Profile;
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;
	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;
}CMD_MSJog;

// MI_JOG Move Struct Define -> Only DRS-03xx series
typedef struct CMD_MIJog
{
	short Value;
	unsigned char Stop;
	unsigned char InfiniteTurn;
	unsigned char LED;
	unsigned char NoAction;
	unsigned char ID;
	unsigned char PlayTime_ms;
	unsigned char Profile;
	unsigned char Padding1;
	unsigned char Padding2;
	unsigned char Padding3;
	unsigned char Padding4;
	unsigned char Padding5;
	unsigned char Padding6;
	unsigned char Padding7;
}CMD_MIJog;
/************************************************************************************************************************************************************/

//////////////////////////////수신 상태 결과 값//////////////////////////////
enum {
	DRS_RXWAITING,
	DRS_RXCOMPLETE,
	DRS_HEADERNOTFOUND,
	DRS_INVALIDSIZE,
	DRS_UNKNOWNCMD,
	DRS_INVALIDID,
	DRS_CHKSUMERROR,
	DRS_RXTIMEOUT
}DrsRxStatus;

#pragma pack(pop)

typedef void CALLING_CONVENTION stat_ack(unsigned char sid, unsigned char status_error, unsigned char status_detail);

typedef stat_ack eep_write_ack;
typedef stat_ack eep_map_write_ack;
typedef void CALLING_CONVENTION eep_read_ack(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail);
typedef void CALLING_CONVENTION eep_map_read_ack(unsigned char sid, unsigned char address, unsigned char length, EEPRegisterMap map, unsigned char status_error, unsigned char status_detail);

typedef stat_ack ram_write_ack;
typedef stat_ack ram_map_write_ack;
typedef void CALLING_CONVENTION ram_read_ack(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail);
typedef void CALLING_CONVENTION ram_map_read_ack(unsigned char sid, unsigned char address, unsigned char length, RAMRegisterMap map, unsigned char status_error, unsigned char status_detail);

typedef stat_ack i_jog_ack;
typedef stat_ack s_jog_ack;
typedef stat_ack mi_jog_ack;
typedef stat_ack ms_jog_ack;
typedef stat_ack rollback_ack;
typedef stat_ack reboot_ack;


API int CALLING_CONVENTION get_eep_register_size(unsigned char eep_reg);
API int CALLING_CONVENTION get_ram_register_size(unsigned char ram_reg);


API int CALLING_CONVENTION set_eep_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value);
API int CALLING_CONVENTION set_eep_map_write_cmd(void* buffer, unsigned char sid, EEPRegisterMap map, unsigned char address, unsigned char count);
API int CALLING_CONVENTION set_eep_read_cmd(void* buffer, unsigned char sid, unsigned char address);
API int CALLING_CONVENTION set_eep_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count);

API int CALLING_CONVENTION set_ram_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value);
API int CALLING_CONVENTION set_ram_map_write_cmd(void* buffer, unsigned char sid, RAMRegisterMap map, unsigned char address, unsigned char count);
API int CALLING_CONVENTION set_ram_read_cmd(void* buffer, unsigned char sid, unsigned char address);
API int CALLING_CONVENTION set_ram_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count);


API int CALLING_CONVENTION set_i_jog_cmd(void* buffer, unsigned char sid, IJog* jogs, int count);
API int CALLING_CONVENTION set_s_jog_cmd(void* buffer, unsigned char sid, unsigned char time_ms, SJog* jogs, int count);
API int CALLING_CONVENTION set_mi_jog_cmd(void* buffer, unsigned char sid, MIJog* jogs, int count, int turn, bool bMulti);
API int CALLING_CONVENTION set_ms_jog_cmd(void* buffer, unsigned char sid, unsigned char time_ms, MSJog* jogs, int count, int turn, bool bMulti);
API int CALLING_CONVENTION set_stat_cmd(void* buffer, unsigned char sid);
API int CALLING_CONVENTION set_rollback_cmd(void* buffer, unsigned char sid, unsigned char id_skip, unsigned char baud_skip);
API int CALLING_CONVENTION set_reboot_cmd(void* buffer, unsigned char sid);
//Add CALSET
API int CALLING_CONVENTION set_calset_cmd(void* buffer, unsigned char sid);


API void CALLING_CONVENTION regist_ack_callback_eep_write(eep_write_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_eep_map_write(eep_map_write_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_eep_read(eep_read_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_eep_map_read(eep_map_read_ack* callback);

API void CALLING_CONVENTION regist_ack_callback_ram_write(ram_write_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_ram_map_write(ram_map_write_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_ram_read(ram_read_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_ram_map_read(ram_map_read_ack* callback);

API void CALLING_CONVENTION regist_ack_callback_i_jog(i_jog_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_s_jog(s_jog_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_mi_jog(mi_jog_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_ms_jog(ms_jog_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_stat(stat_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_rollback(rollback_ack* callback);
API void CALLING_CONVENTION regist_ack_callback_reboot(reboot_ack* callback);

API int CALLING_CONVENTION parse(void* buffer, int length, int* pos);

#endif /* HERKULEX_PROTOCOL_H_ */