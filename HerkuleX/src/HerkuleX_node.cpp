/*
HerkuleX_node
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <list>
using namespace std;
#include <pthread.h>

#include "HerkuleX_node.h"
#include "HerkuleX/MsgHerkuleX_RAM.h" //MSG
#include "HerkuleX/MsgHerkuleX_EEP.h" //MSG
#include "HerkuleX/HerkuleX_Info_RAM.h" //MSG_Array
#include "HerkuleX/HerkuleX_Info_EEP.h" //MSG_Array

#include "HerkuleX/HerkuleX_RegisterCommand.h" //SRV
#include "HerkuleX/HerkuleX_PositionMove.h" //SRV
#include "HerkuleX/HerkuleX_VelocityMove.h" //SRV
#include "HerkuleX/HerkuleX_SJOG_Move.h" //SRV
#include "HerkuleX/HerkuleX_IJOG_Move.h" //SRV

serial::Serial ser;
pthread_t p_thread;

/*********************************************************************************************************************************/
unsigned char EEP_REG_SIZE[] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0 };
unsigned char RAM_REG_SIZE[] = { 1,1,1,1,1,1,1,1,1,1,1,1,2,0,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0 };

int m_iModel = 0; //0: DRS-0101, 1: DRS-0102, 2: DRS-0201, 3: DRS-0301, 4: DRS-0302, 5: DRS-0303, 6: DRS-0401, 7: DRS-0402, 8: DRS-0601, 9: DRS-0602
// 사용할 모터 id 번호. ex)1 ~ 253까지
unsigned char szIDs[] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
						  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,
						  0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,
						  0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,
						  0x3d,0x3e,0x3f,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,
						  0x4c,0x4d,0x4e,0x4f,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,
						  0x5b,0x5c,0x5d,0x5e,0x5f,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
						  0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,
						  0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,
						  0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x96,
						  0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,
						  0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
						  0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,0xc0,0xc1,0xc2,0xc3,
						  0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,
						  0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,0xe0,0xe1,
						  0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,0xf0,
						  0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe };

#define EEPRegSize(addr) (EEP_REG_SIZE[addr])
#define RAMRegSize(addr) (RAM_REG_SIZE[addr])

SJog			sjogs[255];
IJog			ijogs[255];
MSJog			msjogs[255]; //Multi Turn Add_Only New HerkuleX series
MIJog			mijogs[255]; //Multi Turn Add_Only New HerkuleX series
RAMRegisterMap	RAM[255];
EEPRegisterMap	EEP[255];

//S_JOG CMD
CMD_SJog		sjogArr[255];
//I_JOG CMD
CMD_IJog		ijogArr[255];

unsigned char szSendBuffer[1024] = { 0, };	// 전송 버퍼
int	          nPacketLength = 0;
unsigned char ucValue = 0;

bool m_bView_Flag = false; //Packet View Flag

bool m_bRAM_ReadAll_Flag = false;
bool m_bEEP_ReadAll_Flag = false;

//test//
int iRAM_callback_cnt = 0;
int m_iTotal_Axis = 1;

/*Array_RAM&EEP msg*/
HerkuleX::HerkuleX_Info_RAM RAM_Array;
HerkuleX::HerkuleX_Info_EEP EEP_Array;
ros::Publisher RAM_Array_pub[255];
ros::Publisher EEP_Array_pub[255];

/*RAM & EEP msg _ Publish************************/
HerkuleX::MsgHerkuleX_RAM RAM_msg;
HerkuleX::MsgHerkuleX_EEP EEP_msg;
//ros::Publisher RAM_pub;
//ros::Publisher EEP_pub;
/*Command srv _ Subscriber Service************************/
HerkuleX::HerkuleX_RegisterCommand Register_cmd;
HerkuleX::HerkuleX_PositionMove Position_cmd;
HerkuleX::HerkuleX_VelocityMove Velocity_cmd;
ros::ServiceServer Register_service;
ros::ServiceServer PositionCmd_service;
ros::ServiceServer VelocityCmd_service;
ros::ServiceServer SJOGCmd_service;
ros::ServiceServer IJOGCmd_service;
/*****************************/
std_msgs::UInt8MultiArray  r_buffer;
std_msgs::UInt8MultiArray  w_buffer;
ros::Publisher write_pub;
//ros::Subscriber write_sub;
ros::Publisher read_pub;


void CALLING_CONVENTION EEPWriteAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("EEPWriteAckCallback...");
}

void CALLING_CONVENTION EEPReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail)
{
	//printf("EEPReadAckCallback...");
}

void CALLING_CONVENTION RAMWriteAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("RAMWriteAckCallback...");
}

void CALLING_CONVENTION RAMReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail)
{
	//printf("RAMReadAckCallback...");
}

void CALLING_CONVENTION EEPMapReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, EEPRegisterMap map, unsigned char status_error, unsigned char status_detail)
{
	//memset(&EEP_Array, 0, sizeof(EEP_Array));
	//memset(EEP_Array, 0, sizeof(EEP_Array));

	if(m_bEEP_ReadAll_Flag)
	{
		memcpy(&(EEP[sid]), &map, sizeof(EEPRegisterMap));
		memcpy(&EEP_msg, &map, sizeof(EEPRegisterMap));

		m_bEEP_ReadAll_Flag = false;
	}
	else
	{
		switch (address)
		{
		case 0:
			EEP_msg.ModelNo1 = EEP[sid].ucModelNo1 = map.ucModelNo1;
			break;
		case 1:
			EEP_msg.ModelNo2 = EEP[sid].ucModelNo2 = map.ucModelNo2;
			break;
		case 2:
			EEP_msg.Version1 = EEP[sid].ucVersion1 = map.ucVersion1;
			break;
		case 3:
			EEP_msg.Version2 = EEP[sid].ucVersion2 = map.ucVersion2;
			break;
		case 4:
			EEP_msg.BaudRate = EEP[sid].ucBaudRate = map.ucBaudRate;
			break;
		case 5:
			EEP_msg.Reaserved1 = EEP[sid].ucReaserved1 = map.ucReaserved1;
			break;
		case 6:
			EEP_msg.ID = EEP[sid].ucID = map.ucID;
			break;
		case 7:
			EEP_msg.AckPolicy = EEP[sid].ucAckPolicy = map.ucAckPolicy;
			break;
		case 8:
			EEP_msg.AlarmLEDPolicy = EEP[sid].ucAlarmLEDPolicy = map.ucAlarmLEDPolicy;
			break;
		case 9:
			EEP_msg.TorquePolicy = EEP[sid].ucTorquePolicy = map.ucTorquePolicy;
			break;
		case 10:
			EEP_msg.Reserved2 = EEP[sid].ucReserved2 = map.ucReserved2;
			break;
		case 11:
			EEP_msg.MaxTemperature = EEP[sid].ucMaxTemperature = map.ucMaxTemperature;
			break;
		case 12:
			EEP_msg.MinVoltage = EEP[sid].ucMinVoltage = map.ucMinVoltage;
			break;
		case 13:
			EEP_msg.MaxVoltage = EEP[sid].ucMaxVoltage = map.ucMaxVoltage;
			break;
		case 14:
			EEP_msg.AccelerationRatio = EEP[sid].ucAccelerationRatio = map.ucAccelerationRatio;
			break;
		case 15:
			EEP_msg.MaxAccelerationTime = EEP[sid].ucMaxAccelerationTime = map.ucMaxAccelerationTime;
			break;
		case 16:
			EEP_msg.DeadZone = EEP[sid].ucDeadZone = map.ucDeadZone;
			break;
		case 17:
			EEP_msg.SaturatorOffset = EEP[sid].ucSaturatorOffset = map.ucSaturatorOffset;
			break;
		case 18:
			EEP_msg.SaturatorSlope = EEP[sid].usSaturatorSlope = map.usSaturatorSlope;
			break;
		case 20:
			EEP_msg.PWMOffset = EEP[sid].cPWMOffset = map.cPWMOffset;
			break;
		case 21:
			EEP_msg.MinPWM = EEP[sid].ucMinPWM = map.ucMinPWM;
			break;
		case 22:
			EEP_msg.MaxPWM = EEP[sid].usMaxPWM = map.usMaxPWM;
			break;
		case 24:
			EEP_msg.OverloadPWMThreshold = EEP[sid].usOverloadPWMThreshold = map.usOverloadPWMThreshold;
			break;
		case 26:
			EEP_msg.MinPosition = EEP[sid].usMinPosition = map.usMinPosition;
			break;
		case 28:
			EEP_msg.MaxPosition = EEP[sid].usMaxPosition = map.usMaxPosition;
			break;
		case 30: 
			EEP_msg.PositionKp = EEP[sid].usPositionKp = map.usPositionKp;
			break;
		case 32:
			EEP_msg.PositionKd = EEP[sid].usPositionKd = map.usPositionKd;
			break;
		case 34:
			EEP_msg.PositionKi = EEP[sid].usPositionKi = map.usPositionKi;
			break;
		case 36:
			EEP_msg.PositionFeedforward1stGain = EEP[sid].usPositionFeedforward1stGain = map.usPositionFeedforward1stGain;
			break;
		case 38: //usVelocityKp
			EEP_msg.PositionFeedforward2ndGain = EEP[sid].usPositionFeedforward2ndGain = map.usPositionFeedforward2ndGain;
			break;
		case 40: 
			EEP_msg.VelocityKd = EEP[sid].usVelocityKd = map.usVelocityKd;
			break;
		case 42: 
			EEP_msg.VelocityKi = EEP[sid].usVelocityKi = map.usVelocityKi;
			break;
		case 44:
			EEP_msg.LEDBlinkPeriod = EEP[sid].ucLEDBlinkPeriod = map.ucLEDBlinkPeriod;
			break;
		case 45:
			EEP_msg.ADCFaultCheckPeriod = EEP[sid].ucADCFaultCheckPeriod = map.ucADCFaultCheckPeriod;
			break;
		case 46:
			EEP_msg.PacketGarbageCheckPeriod = EEP[sid].ucPacketGarbageCheckPeriod = map.ucPacketGarbageCheckPeriod;
			break;
		case 47:
			EEP_msg.StopDetectionPeriod = EEP[sid].ucStopDetectionPeriod = map.ucStopDetectionPeriod;
			break;
		case 48:
			EEP_msg.OverloadDetectionPeriod = EEP[sid].ucOverloadDetectionPeriod = map.ucOverloadDetectionPeriod;
			break;
		case 49:
			EEP_msg.StopThreshold = EEP[sid].ucStopThreshold = map.ucStopThreshold;
			break;
		case 50:
			EEP_msg.InpositionMargin = EEP[sid].ucInpositionMargin = map.ucInpositionMargin;
			break;
		case 51:
			EEP_msg.Reserved5 = EEP[sid].ucReserved5 = map.ucReserved5;
			break;
		case 52:
			EEP_msg.CalibrationDifference_L = EEP[sid].cCalibrationDifference_L = map.cCalibrationDifference_L;
			break;
		case 53:
			EEP_msg.CalibrationDifference_H = EEP[sid].cCalibrationDifference_H = map.cCalibrationDifference_H;
			//memcpy(EEP_msg, EEP, sizeof(EEP_msg));
			break;
		default:
			printf("Error: Not defined Adress... \n");
			break;
		}
	}

	//EEP_pub.publish(EEP_msg);
	EEP_Array.index = sid;
	EEP_Array.pHerkuleX_EEP.push_back(EEP_msg);
	EEP_Array_pub[sid].publish(EEP_Array);
	EEP_Array.pHerkuleX_EEP.clear();

}


void CALLING_CONVENTION RAMMapReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, RAMRegisterMap map, unsigned char status_error, unsigned char status_detail)
{
	//memset(&RAM_Array, 0, sizeof(RAM_Array));
	//memset(RAM_Array, 0, sizeof(RAM_Array));

	if(m_bRAM_ReadAll_Flag)
	{
		memcpy(&(RAM[sid]), &map, sizeof(RAMRegisterMap));
		memcpy(&RAM_msg, &map, sizeof(RAMRegisterMap));

		switch(m_iModel)
		{
			case 0: //0101
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
				break;
			case 1: //0102
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
				break;
			case 2: //0201
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
				break;
			case 3: //0301
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
				break;
			case 4: //0302
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
				break;
			case 5: //0303
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
				break;
			case 6: //0401
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
				break;
			case 7: //0402
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
				break;
			case 8: //0601
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
				break;
			case 9: //0602
				RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
				break;
		}

		m_bRAM_ReadAll_Flag = false;
	}
	else
	{
		switch (address)
		{
		case 0: //ID
			RAM_msg.ID = RAM[sid].ucID = map.ucID;
			break;
		case 1: //ACK POLICY
			RAM_msg.AckPolicy = RAM[sid].ucAckPolicy = map.ucAckPolicy;
			break;
		case 2:
			RAM_msg.AlarmLEDPolicy = RAM[sid].ucAlarmLEDPolicy = map.ucAlarmLEDPolicy;
			break;
		case 3:
			RAM_msg.TorquePolicy = RAM[sid].ucTorquePolicy = map.ucTorquePolicy;
			break;
		case 4:
			RAM_msg.Reserved2 = RAM[sid].ucReserved2 = map.ucReserved2;
			break;
		case 5:
			RAM_msg.MaxTemperature = RAM[sid].ucMaxTemperature = map.ucMaxTemperature;
			break;
		case 6:
			RAM_msg.MinVoltage = RAM[sid].ucMinVoltage = map.ucMinVoltage;
			break;
		case 7:
			RAM_msg.MaxVoltage = RAM[sid].ucMaxVoltage = map.ucMaxVoltage;
			break;
		case 8:
			RAM_msg.AccelerationRatio = RAM[sid].ucAccelerationRatio = map.ucAccelerationRatio;
			break;
		case 9:
			RAM_msg.MaxAccelerationTime = RAM[sid].ucMaxAccelerationTime = map.ucMaxAccelerationTime;
			break;
		case 10:
			RAM_msg.DeadZone = RAM[sid].ucDeadZone = map.ucDeadZone;
			break;
		case 11:
			RAM_msg.SaturatorOffset = RAM[sid].ucSaturatorOffset = map.ucSaturatorOffset;
			break;
		case 12:
			RAM_msg.SaturatorSlope = RAM[sid].usSaturatorSlope = map.usSaturatorSlope;
			break;
		case 14:
			RAM_msg.PWMOffset = RAM[sid].cPWMOffset = map.cPWMOffset;
			break;
		case 15:
			RAM_msg.MinPWM = RAM[sid].ucMinPWM = map.ucMinPWM;
			break;
		case 16:
			RAM_msg.MaxPWM = RAM[sid].usMaxPWM = map.usMaxPWM;
			break;
		case 18:
			RAM_msg.OverloadPWMThreshold = RAM[sid].usOverloadPWMThreshold = map.usOverloadPWMThreshold;
			break;
		case 20: //GET_MinPosition
			RAM_msg.MinPosition = RAM[sid].usMinPosition = map.usMinPosition;
			break;
		case 22: //GET_MaxPosition
			RAM_msg.MaxPosition = RAM[sid].usMaxPosition = map.usMaxPosition;
			break;
		case 24:
			RAM_msg.PositionKp = RAM[sid].usPositionKp = map.usPositionKp;
			break;
		case 26:
			RAM_msg.PositionKd = RAM[sid].usPositionKd = map.usPositionKd;
			break;
		case 28:
			RAM_msg.PositionKi = RAM[sid].usPositionKi = map.usPositionKi;
			break;
		case 30:
			RAM_msg.PositionFeedforward1stGain = RAM[sid].usPositionFeedforward1stGain = map.usPositionFeedforward1stGain;
			break;
		case 32:
			RAM_msg.PositionFeedforward2ndGain = RAM[sid].usPositionFeedforward2ndGain = map.usPositionFeedforward2ndGain;
			break;
		case 34:
			RAM_msg.VelocityKd = RAM[sid].usVelocityKd = map.usVelocityKd;
			break;
		case 36:
			RAM_msg.VelocityKi = RAM[sid].usVelocityKi = map.usVelocityKi;
			break;
		case 38:
			RAM_msg.LEDBlinkPeriod = RAM[sid].ucLEDBlinkPeriod = map.ucLEDBlinkPeriod;
			break;
		case 39:
			RAM_msg.ADCFaultCheckPeriod = RAM[sid].ucADCFaultCheckPeriod = map.ucADCFaultCheckPeriod;
			break;
		case 40:
			RAM_msg.PacketGarbageCheckPeriod = RAM[sid].ucPacketGarbageCheckPeriod = map.ucPacketGarbageCheckPeriod;
			break;
		case 41:
			RAM_msg.StopDetectionPeriod = RAM[sid].ucStopDetectionPeriod = map.ucStopDetectionPeriod;
			break;
		case 42:
			RAM_msg.OverloadDetectionPeriod = RAM[sid].ucOverloadDetectionPeriod = map.ucOverloadDetectionPeriod;
			break;
		case 43:
			RAM_msg.StopThreshold = RAM[sid].ucStopThreshold = map.ucStopThreshold;
			break;
		case 44:
			RAM_msg.InpositionMargin = RAM[sid].ucInpositionMargin = map.ucInpositionMargin;
			break;
		case 45:
			RAM_msg.Turn = RAM[sid].ucTurn = map.ucTurn;
			break;
		case 46:
			RAM_msg.CalibrationDifference_L = RAM[sid].cCalibrationDifference_L = map.cCalibrationDifference_L;
			break;
		case 47:
			RAM_msg.CalibrationDifference_H = RAM[sid].cCalibrationDifference_H = map.cCalibrationDifference_H;
			break;
		case 48:
			RAM_msg.StatusError = RAM[sid].ucStatusError = map.ucStatusError;
			break;
		case 49:
			RAM_msg.StatusDetail = RAM[sid].ucStatusDetail = map.ucStatusDetail;
			break;
		case 50:
			RAM_msg.Reserved7 = RAM[sid].ucReserved7 = map.ucReserved7;
			break;
		case 51:
			RAM_msg.Reserved8 = RAM[sid].ucReserved8 = map.ucReserved8;
			break;
		case 52:
			RAM_msg.TorqueControl = RAM[sid].ucTorqueControl = map.ucTorqueControl;
			break;
		case 53:
			RAM_msg.LEDControl = RAM[sid].ucLEDControl = map.ucLEDControl;
			break;
		case 54:
			RAM_msg.Voltage = RAM[sid].ucVoltage = map.ucVoltage;
			break;
		case 55:
			RAM_msg.Temperature = RAM[sid].ucTemperature = map.ucTemperature;
			break;
		case 56:
			RAM_msg.CurrentControlMode = RAM[sid].ucCurrentControlMode = map.ucCurrentControlMode;
			break;
		case 57:
			RAM_msg.Tick = RAM[sid].ucTick = map.ucTick;
			break;
		case 58:
			switch(m_iModel)
			{
				case 0: //DRS-0101
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
					break;
				case 1: //DRS-0102
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
					break;
				case 2: //DRS-0201
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
					break;
				case 3: //DRS-0301
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
					break;
				case 4: //DRS-0302
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
					break;
				case 5: //DRS-0303
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition;
					break;
				case 6: //DRS-0401
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
					break;
				case 7: //DRS-0402
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
					break;
				case 8: //DRS-0601
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
					break;
				case 9: //DRS-0602
					RAM_msg.CalibratedPosition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
					break;
			}
			break;
		case 60:
			RAM_msg.AbsolutePosition = RAM[sid].usAbsolutePosition = map.usAbsolutePosition;
			break;
		case 62:
			RAM_msg.DifferentialPosition = RAM[sid].sDifferentialPosition = map.sDifferentialPosition;
			break;
		case 64:
			RAM_msg.PWM = RAM[sid].usPWM = map.usPWM;
			break;
		case 66:
			RAM_msg.Reserved9 = RAM[sid].usReserved9 = map.usReserved9;
			break;
		case 68:
			RAM_msg.AbsoluteGoalPosition = RAM[sid].usAbsoluteGoalPosition = map.usAbsoluteGoalPosition;
			break;
		case 70:
			RAM_msg.AbsoluteDesiredTrajectoryPosition = RAM[sid].usAbsoluteDesiredTrajectoryPosition = map.usAbsoluteDesiredTrajectoryPosition;
			break;
		case 72:
			RAM_msg.DesiredVelocity = RAM[sid].sDesiredVelocity = map.sDesiredVelocity;
			//memcpy(RAM_msg, RAM, sizeof(RAM_msg));
			//memcpy(RAM, &map, sizeof(RAMRegisterMap));
			break;
		default:
			printf("Error: Not defined Adress... \n");
			break;
		}
	}

	//RAM_pub.publish(RAM_msg);
	RAM_Array.index = sid;
	RAM_Array.pHerkuleX_RAM.push_back(RAM_msg);
	RAM_Array_pub[sid].publish(RAM_Array);
	RAM_Array.pHerkuleX_RAM.clear();
}

void CALLING_CONVENTION StatAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("StatAckCallback...");
}

void CALLING_CONVENTION IJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("IJogAckCallback...");
}

void CALLING_CONVENTION SJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("SJogAckCallback...");
}

void CALLING_CONVENTION MIJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("MIJogAckCallback...");
}

void CALLING_CONVENTION MSJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("MSJogAckCallback...");
}

void CALLING_CONVENTION RollbackAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("RollbackAckCallback...");
}

void CALLING_CONVENTION RebootAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
	//printf("RebootAckCallback...");
}

 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DrsData* get_packet_data(void* packet)
{
	DrsPacket* pPacket = (DrsPacket*)packet;

	return &pPacket->unData;
}

unsigned char set_packet(void* buffer, unsigned char id, unsigned char cmd, int data_length)
{
	if (buffer == 0)
		return MIN_PACKET_SIZE + data_length;

	DrsPacket* packet = (DrsPacket*)buffer;
	unsigned char i;

	//헤더 입력
	packet->ucHeader = HEADER;
	packet->ucPacketSize = (unsigned char)(MIN_PACKET_SIZE + data_length);
	packet->ucChipID = id;
	packet->ucCmd = cmd;

	//CheckSum 계산 후 입력
	packet->ucCheckSum1 = packet->ucPacketSize ^ packet->ucChipID ^ packet->ucCmd;
	for (i = 0; i < (packet->ucPacketSize - MIN_PACKET_SIZE); i++)
		packet->ucCheckSum1 ^= packet->unData.ucData[i];

	packet->ucCheckSum2 = (unsigned char)(~(packet->ucCheckSum1));
	packet->ucCheckSum1 &= CHKSUM_MASK;
	packet->ucCheckSum2 &= CHKSUM_MASK;

	return packet->ucPacketSize;
}

DrsPacket* get_ack_packet(void* buffer, int size, int* pos)
{
	int i, j;
	unsigned char check1, check2;
	unsigned char* buff = (unsigned char*)buffer;

	if (size < MIN_ACK_PACKET_SIZE)
		return 0;

	for (i = 0; i <= size - MIN_ACK_PACKET_SIZE; i++)
	{
		DrsPacket* packet = (DrsPacket*)(buff + i);
		if (packet->ucHeader != HEADER)
			continue;
		if (packet->ucPacketSize > size - i)
			continue;

		check1 = packet->ucPacketSize ^ packet->ucChipID ^ packet->ucCmd;
		for (j = 0; j < (packet->ucPacketSize - MIN_PACKET_SIZE); j++)
			check1 ^= packet->unData.ucData[j];

		check2 = (unsigned char)(~(check1));
		check1 &= CHKSUM_MASK;
		check2 &= CHKSUM_MASK;
		if (packet->ucCheckSum1 != check1 || packet->ucCheckSum2 != check2)
			continue;

		*pos = i + packet->ucPacketSize;

		return packet;
	}

	return 0;
}

DrsStatData* get_ack_status(DrsPacket* packet)
{
	DrsStatData* pStat = 0;
	if (packet->ucCmd == CMD_EEP_READ_ACK || packet->ucCmd == CMD_RAM_READ_ACK)
	{
		pStat = (DrsStatData*)& packet->unData.ucData[packet->unData.stRWData.ucLen];
	}
	else
	{
		pStat = &packet->unData.stStatData;
	}

	return pStat;
}

 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int CALLING_CONVENTION set_ram_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value)
{
	DrsRWData* data = 0;
	unsigned char size = RAMRegSize(address);

	if (address > RAM_DESIRED_VELOCITY)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;
	memcpy(data->ucData, value, size);

	return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));
}

int CALLING_CONVENTION set_ram_map_write_cmd(void* buffer, unsigned char sid, RAMRegisterMap map, unsigned char address, unsigned char count)
{
	DrsRWData* data = 0;
	unsigned char size = 0;
	unsigned char regSize = 0;
	for (unsigned char i = address; i <= RAM_LAST && count > 0; i++)
	{
		regSize = RAMRegSize(i);
		size += regSize;
		if (regSize > 0)
			count--;
	}

	if (address > RAM_DESIRED_VELOCITY)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;
	memcpy(data->ucData, (char*)& map + address, size);

	return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));
}

int CALLING_CONVENTION set_ram_read_cmd(void * buffer, unsigned char sid, unsigned char address)
{
	unsigned char size = RAMRegSize(address);
	DrsRWData* data = 0;

	if (address > RAM_LAST)
		return -1;
	if (buffer == 0) 
		return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;

	return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));
}

int CALLING_CONVENTION set_ram_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count)
{
	DrsRWData* data = 0;
	unsigned char size = 0;
	unsigned char regSize = 0;
	for (unsigned char i = address; i <= RAM_LAST && count > 0; i++)
	{
		regSize = RAMRegSize(i);
		size += regSize;
		if (regSize > 0) count--;
	}

	if (address > RAM_LAST)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;

	return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
API int CALLING_CONVENTION get_eep_register_size(unsigned char eep_reg)
{
	return EEPRegSize(eep_reg);
}

API int CALLING_CONVENTION get_ram_register_size(unsigned char ram_reg)
{
	return RAMRegSize(ram_reg);
}

int CALLING_CONVENTION set_eep_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value)
{
	DrsRWData* data = 0;
	unsigned char size = EEPRegSize(address);

	if (address > EEP_LAST)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;
	memcpy(data->ucData, value, size);

	return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));
}

int CALLING_CONVENTION set_eep_map_write_cmd(void* buffer, unsigned char sid, EEPRegisterMap map, unsigned char address, unsigned char count)
{
	DrsRWData* data = 0;
	unsigned char size = 0;
	unsigned char regSize = 0;
	for (unsigned char i = address; i <= EEP_LAST && count > 0; i++)
	{
		regSize = EEPRegSize(i);
		size += regSize;
		if (regSize > 0)
			count--;
	}

	if (address > EEP_LAST)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));

	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;
	memcpy(data->ucData, (char*)& map + address, size);

	return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));
}

int CALLING_CONVENTION set_eep_read_cmd(void* buffer, unsigned char sid, unsigned char address)
{
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));

	if (address > EEP_CALIBRATION_DIFFERENCE)
		return -1;

	unsigned char len = 0;
	for (unsigned char i = address; i <= EEP_CALIBRATION_DIFFERENCE; i++)
	{
		len += EEPRegSize(i);
	}

	DrsRWData* data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = len;

	return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));
}

int CALLING_CONVENTION set_eep_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count)
{
	DrsRWData* data = 0;
	unsigned char size = 0;
	unsigned char regSize = 0;
	for (unsigned char i = address; i <= EEP_LAST && count > 0; i++)
	{
		regSize = EEPRegSize(i);
		size += regSize;
		if (regSize > 0) count--;
	}

	if (address > EEP_LAST)
		return -1;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));


	data = &(get_packet_data(buffer)->stRWData);
	data->ucAddress = address;
	data->ucLen = size;

	//printf("count %d, size : %d\n", count, size);
	return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int CALLING_CONVENTION set_s_jog_cmd(void* buffer, unsigned char sid, unsigned char time_ms, SJog* jogs, int count)
{
	DrsSJogData* data = 0;
	DrsSJog* sjog = 0;

	if (buffer == 0)
		return set_packet(buffer, sid, CMD_S_JOG, (int)sizeof(DrsSJogData) + (int)sizeof(DrsSJog) * count);

	data = &(get_packet_data(buffer)->stSJogData);
	sjog = data->stSJog;
	for (int i = 0; i < count; i++)
	{
		if (jogs[i].InfiniteTurn)
		{
			sjog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
			sjog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
			sjog[i].stJog.Infinite.reserved = 0;
		}
		else 
		{
			sjog[i].stJog.Position.iValue = (jogs[i].Value);
		}

		sjog[i].stSet.ucStopFlag = jogs[i].Stop;
		sjog[i].stSet.ucMode = jogs[i].InfiniteTurn;
		sjog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
		sjog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
		sjog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
		sjog[i].stSet.ucJogInvalid = jogs[i].NoAction;
		sjog[i].stSet.ucProfile = jogs[i].Profile;

		sjog[i].ucId = jogs[i].ID;
	}
	data->ucPlayTime = time_ms;
	return set_packet(buffer, sid, CMD_S_JOG, (int)(sizeof(DrsSJogData) + sizeof(DrsSJog) * (unsigned int)count));
}

int CALLING_CONVENTION set_i_jog_cmd(void* buffer, unsigned char sid, IJog* jogs, int count)
{
	DrsIJog* ijog = 0;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_I_JOG, (int)sizeof(DrsIJog) * count);

	ijog = get_packet_data(buffer)->stIJog;
	for (int i = 0; i < count; i++)
	{
		if (jogs[i].InfiniteTurn)
		{
			ijog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
			ijog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
			ijog[i].stJog.Infinite.reserved = 0;
		}
		else 
		{
			ijog[i].stJog.Position.iValue = (jogs[i].Value);
		}

		ijog[i].stSet.ucStopFlag = jogs[i].Stop;
		ijog[i].stSet.ucMode = jogs[i].InfiniteTurn;
		ijog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
		ijog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
		ijog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
		ijog[i].stSet.ucJogInvalid = jogs[i].NoAction;
		ijog[i].stSet.ucProfile = jogs[i].Profile;

		ijog[i].ucId = jogs[i].ID;
		ijog[i].ucPlayTime = jogs[i].PlayTime_ms;
	}

	return set_packet(buffer, sid, CMD_I_JOG, (int)sizeof(DrsIJog) * count);
}

int CALLING_CONVENTION set_ms_jog_cmd(void* buffer, unsigned char sid, unsigned char time_ms, MSJog* jogs, int count, int turn, bool bMulti)
{
	DrsMSJogData* data = 0;
	DrsMSJog* msjog = 0;

	if (buffer == 0)
		return set_packet(buffer, sid, CMD_MS_JOG, (int)sizeof(DrsMSJogData) + (int)sizeof(DrsMSJog) * count);

	data = &(get_packet_data(buffer)->stMSJogData);
	msjog = data->stMSJog;
	for (int i = 0; i < count; i++)
	{
		if (jogs[i].InfiniteTurn)
		{
			msjog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
			msjog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
			//Add//
			msjog[i].stJog2.Turn.uiValue = turn; //Turn count//

			if (bMulti)
			{
				msjog[i].stSet.reserved = 1;
			}
			else
			{
				msjog[i].stSet.reserved = 0;
			}
		}
		else
		{
			msjog[i].stJog.Position.iValue = (jogs[i].Value);
			//msjog[i].stJog.Position.uiValue = abs(jogs[i].Value);
			//Add//
			msjog[i].stJog2.Turn.uiValue = turn; //Turn count//

			if (bMulti)
			{
				msjog[i].stSet.reserved = 1;
			}
			else
			{
				msjog[i].stSet.reserved = 0;
			}
		}

		msjog[i].stSet.ucStopFlag = jogs[i].Stop;
		msjog[i].stSet.ucMode = jogs[i].InfiniteTurn;
		msjog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
		msjog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
		msjog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
		msjog[i].stSet.ucJogInvalid = jogs[i].NoAction;
		msjog[i].stSet.ucProfile = jogs[i].Profile;

		msjog[i].ucId = jogs[i].ID;
	}
	data->ucPlayTime = time_ms;
	return set_packet(buffer, sid, CMD_MS_JOG, (int)(sizeof(DrsMSJogData) + sizeof(DrsMSJog) * (unsigned int)count));
}

int CALLING_CONVENTION set_mi_jog_cmd(void* buffer, unsigned char sid, MIJog* jogs, int count, int turn, bool bMulti)
{
	DrsMIJog* mijog = 0;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_MI_JOG, (int)sizeof(DrsMIJog) * count);

	mijog = get_packet_data(buffer)->stMIJog;
	for (int i = 0; i < count; i++)
	{
		if (jogs[i].InfiniteTurn)
		{
			mijog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
			mijog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
			//Add//
			mijog[i].stJog2.Turn.uiValue = turn; //Turn count//

			if (bMulti)
			{
				mijog[i].stSet.reserved = 1;
			}
			else
			{
				mijog[i].stSet.reserved = 0;
			}
		}
		else
		{
			mijog[i].stJog.Position.iValue = (jogs[i].Value);
			//mijog[i].stJog.Position.uiValue = abs(jogs[i].Value);
			//Add//
			mijog[i].stJog2.Turn.uiValue = turn; //Turn count//

			if (bMulti)
			{
				mijog[i].stSet.reserved = 1;
			}
			else
			{
				mijog[i].stSet.reserved = 0;
			}
		}

		mijog[i].stSet.ucStopFlag = jogs[i].Stop;
		mijog[i].stSet.ucMode = jogs[i].InfiniteTurn;
		mijog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
		mijog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
		mijog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
		mijog[i].stSet.ucJogInvalid = jogs[i].NoAction;
		mijog[i].stSet.ucProfile = jogs[i].Profile;

		mijog[i].ucId = jogs[i].ID;
		mijog[i].ucPlayTime = jogs[i].PlayTime_ms;
	}

	return set_packet(buffer, sid, CMD_MI_JOG, (int)sizeof(DrsMIJog) * count);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //RAM & EEP Register Data Command//

bool RAM_RegisterData_Read(unsigned int m_imotor_ID, unsigned int iAddr)
{
	bool bResult = false;

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_ram_read_cmd(szSendBuffer, m_imotor_ID, iAddr);
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;

	return bResult;
}

bool EEP_RegisterData_Read(unsigned int m_imotor_ID, unsigned int iAddr)
{
	bool bResult = false;

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_eep_read_cmd(szSendBuffer, m_imotor_ID, iAddr);
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;

	return bResult;
}

bool RAM_RegisterData_Read_ALL(unsigned int m_imotor_ID, unsigned int iAddr, unsigned char count)
{
	bool bResult = false;

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_ram_map_read_cmd(szSendBuffer, m_imotor_ID, iAddr, count);
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;

	return bResult;
}

bool EEP_RegisterData_Read_ALL(unsigned int m_imotor_ID, unsigned int iAddr, unsigned char count)
{
	bool bResult = false;

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_eep_map_read_cmd(szSendBuffer, m_imotor_ID, iAddr, count);
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;

	return bResult;
}

void RAM_RegisterData_Write(unsigned int m_imotor_ID, unsigned int iAddr, unsigned int cData)
{
	memset(szSendBuffer, 0, sizeof(szSendBuffer));

	switch (iAddr)
	{
	case 0:
		RAM[m_imotor_ID - 1].ucID = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ID, &RAM[m_imotor_ID - 1].ucID);
		break;
	case 1:
		RAM[m_imotor_ID - 1].ucAckPolicy = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ACK_POLICY, &RAM[m_imotor_ID - 1].ucAckPolicy);
		break;
	case 2:
		RAM[m_imotor_ID - 1].ucAlarmLEDPolicy = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ALARM_LED_POLICY, &RAM[m_imotor_ID - 1].ucAlarmLEDPolicy);
		break;
	case 3:
		RAM[m_imotor_ID - 1].ucTorquePolicy = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TORQUE_POLICY, &RAM[m_imotor_ID - 1].ucTorquePolicy);
		break;
	case 5:
		RAM[m_imotor_ID - 1].ucMaxTemperature = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_TEMPERATURE, &RAM[m_imotor_ID - 1].ucMaxTemperature);
		break;
	case 6:
		RAM[m_imotor_ID - 1].ucMinVoltage = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_VOLTAGE, &RAM[m_imotor_ID - 1].ucMinVoltage);
		break;
	case 7:
		RAM[m_imotor_ID - 1].ucMaxVoltage = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_VOLTAGE, &RAM[m_imotor_ID - 1].ucMaxVoltage);
		break;
	case 8:
		RAM[m_imotor_ID - 1].ucAccelerationRatio = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ACCELERATION_RATIO, &RAM[m_imotor_ID - 1].ucAccelerationRatio);
		break;
	case 9:
		RAM[m_imotor_ID - 1].ucMaxAccelerationTime = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_ACCELERATION_TIME, &RAM[m_imotor_ID - 1].ucMaxAccelerationTime);
		break;
	case 10:
		RAM[m_imotor_ID - 1].ucDeadZone = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_DEAD_ZONE, &RAM[m_imotor_ID - 1].ucDeadZone);
		break;
	case 11:
		RAM[m_imotor_ID - 1].ucSaturatorOffset = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_SATURATOR_OFFSET, &RAM[m_imotor_ID - 1].ucSaturatorOffset);
		break;
	case 12:
		RAM[m_imotor_ID - 1].usSaturatorSlope = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_SATURATOR_SLOPE, &RAM[m_imotor_ID - 1].usSaturatorSlope);
		break;
	case 14:
		RAM[m_imotor_ID - 1].cPWMOffset = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_PWM_OFFSET, &RAM[m_imotor_ID - 1].cPWMOffset);
		break;
	case 15:
		RAM[m_imotor_ID - 1].ucMinPWM = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_PWM, &RAM[m_imotor_ID - 1].ucMinPWM);
		break;
	case 16:
		RAM[m_imotor_ID - 1].usMaxPWM = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_PWM, &RAM[m_imotor_ID - 1].usMaxPWM);
		break;
	case 18:
		RAM[m_imotor_ID - 1].usOverloadPWMThreshold = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_OVERLOAD_PWM_THRESHOLD, &RAM[m_imotor_ID - 1].usOverloadPWMThreshold);
		break;
	case 20:
		RAM[m_imotor_ID - 1].usMinPosition = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_POSITION, &RAM[m_imotor_ID - 1].usMinPosition);
		break;
	case 22:
		RAM[m_imotor_ID - 1].usMaxPosition = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_POSITION, &RAM[m_imotor_ID - 1].usMaxPosition);
		break;
	case 24:
		RAM[m_imotor_ID - 1].usPositionKp = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KP, &RAM[m_imotor_ID - 1].usPositionKp);
		break;
	case 26:
		RAM[m_imotor_ID - 1].usPositionKd = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KD, &RAM[m_imotor_ID - 1].usPositionKd);
		break;
	case 28:
		RAM[m_imotor_ID - 1].usPositionKi = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KI, &RAM[m_imotor_ID - 1].usPositionKi);
		break;
	case 30:
		RAM[m_imotor_ID - 1].usPositionFeedforward1stGain = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_FEEDFORWARD_1ST_GAIN, &RAM[m_imotor_ID - 1].usPositionFeedforward1stGain);
		break;
	case 32:
		RAM[m_imotor_ID - 1].usPositionFeedforward2ndGain = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KP, &RAM[m_imotor_ID - 1].usPositionFeedforward2ndGain);
		break;
	case 34:
		RAM[m_imotor_ID - 1].usVelocityKd = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KD, &RAM[m_imotor_ID - 1].usVelocityKd);
		break;
	case 36:
		RAM[m_imotor_ID - 1].usVelocityKi = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KI, &RAM[m_imotor_ID - 1].usVelocityKi);
		break;
	case 38:
		RAM[m_imotor_ID - 1].ucLEDBlinkPeriod = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_LED_BLINK_PERIOD, &RAM[m_imotor_ID - 1].ucLEDBlinkPeriod);
		break;
	case 39:
		RAM[m_imotor_ID - 1].ucADCFaultCheckPeriod = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ADC_FAULT_CHECK_PERIOD, &RAM[m_imotor_ID - 1].ucADCFaultCheckPeriod);
		break;
	case 40:
		RAM[m_imotor_ID - 1].ucPacketGarbageCheckPeriod = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_PACKET_GARBAGE_CHECK_PERIOD, &RAM[m_imotor_ID - 1].ucPacketGarbageCheckPeriod);
		break;
	case 41:
		RAM[m_imotor_ID - 1].ucStopDetectionPeriod = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_STOP_DETECTION_PERIOD, &RAM[m_imotor_ID - 1].ucStopDetectionPeriod);
		break;
	case 42:
		RAM[m_imotor_ID - 1].ucOverloadDetectionPeriod = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_OVELOAD_DETECTION_PERIOD, &RAM[m_imotor_ID - 1].ucOverloadDetectionPeriod);
		break;
	case 43:
		RAM[m_imotor_ID - 1].ucStopThreshold = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_STOP_THRESHOLD, &RAM[m_imotor_ID - 1].ucStopThreshold);
		break;
	case 44:
		RAM[m_imotor_ID - 1].ucInpositionMargin = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_INPOSITION_MARGIN, &RAM[m_imotor_ID - 1].ucInpositionMargin);
		break;
	case 45:
		RAM[m_imotor_ID - 1].ucTurn = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TURN, &RAM[m_imotor_ID - 1].ucTurn);
		break;
	case 46:
		RAM[m_imotor_ID - 1].cCalibrationDifference_L = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_CALIBRATION__DIFFERENCE_L, &RAM[m_imotor_ID - 1].cCalibrationDifference_L);
		break;
	case 47:
		RAM[m_imotor_ID - 1].cCalibrationDifference_H = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_CALIBRATION__DIFFERENCE_H, &RAM[m_imotor_ID - 1].cCalibrationDifference_H);
		break;
	case 52:
		RAM[m_imotor_ID - 1].ucTorqueControl = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TORQUE_CONTROL, &RAM[m_imotor_ID - 1].ucTorqueControl);
		break;
	case 53:
		RAM[m_imotor_ID - 1].ucLEDControl = cData;
		nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_LED_CONTROL, &RAM[m_imotor_ID - 1].ucLEDControl);
		break;
	default:
		printf("This is a read-only address !\n");
		break;

	}

	ser.write(szSendBuffer, nPacketLength);
}

void EEP_RegisterData_Write(unsigned int m_imotor_ID, unsigned int iAddr, unsigned int cData)
{
	memset(szSendBuffer, 0, sizeof(szSendBuffer));

	switch (iAddr)
	{
	case 4:
		EEP[m_imotor_ID - 1].ucBaudRate = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_BAUD_RATE, &EEP[m_imotor_ID - 1].ucBaudRate);
		break;
	case 6:
		EEP[m_imotor_ID - 1].ucID = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ID, &EEP[m_imotor_ID - 1].ucID);
		break;
	case 7:
		EEP[m_imotor_ID - 1].ucAckPolicy = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ACK_POLICY, &EEP[m_imotor_ID - 1].ucAckPolicy);
		break;
	case 8:
		EEP[m_imotor_ID - 1].ucAlarmLEDPolicy = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ALARM_LED_POLICY, &EEP[m_imotor_ID - 1].ucAlarmLEDPolicy);
		break;
	case 9:
		EEP[m_imotor_ID - 1].ucTorquePolicy = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_TORQUE_POLICY, &EEP[m_imotor_ID - 1].ucTorquePolicy);
		break;
	case 11:
		EEP[m_imotor_ID - 1].ucMaxTemperature = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_TEMPERATURE, &EEP[m_imotor_ID - 1].ucMaxTemperature);
		break;
	case 12:
		EEP[m_imotor_ID - 1].ucMinVoltage = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_VOLTAGE, &EEP[m_imotor_ID - 1].ucMinVoltage);
		break;
	case 13:
		EEP[m_imotor_ID - 1].ucMaxVoltage = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_VOLTAGE, &EEP[m_imotor_ID - 1].ucMaxVoltage);
		break;
	case 14:
		EEP[m_imotor_ID - 1].ucAccelerationRatio = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ACCELERATION_RATIO, &EEP[m_imotor_ID - 1].ucAccelerationRatio);
		break;
	case 15:
		EEP[m_imotor_ID - 1].ucMaxAccelerationTime = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_ACCELERATION_TIME, &EEP[m_imotor_ID - 1].ucMaxAccelerationTime);
		break;
	case 16:
		EEP[m_imotor_ID - 1].ucDeadZone = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_DEAD_ZONE, &EEP[m_imotor_ID - 1].ucDeadZone);
		break;
	case 17:
		EEP[m_imotor_ID - 1].ucSaturatorOffset = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_SATURATOR_OFFSET, &EEP[m_imotor_ID - 1].ucSaturatorOffset);
		break;
	case 18:
		EEP[m_imotor_ID - 1].usSaturatorSlope = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_SATURATOR_SLOPE, &EEP[m_imotor_ID - 1].usSaturatorSlope);
		break;
	case 20:
		EEP[m_imotor_ID - 1].cPWMOffset = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_PWM_OFFSET, &EEP[m_imotor_ID - 1].cPWMOffset);
		break;
	case 21:
		EEP[m_imotor_ID - 1].ucMinPWM = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_PWM, &EEP[m_imotor_ID - 1].ucMinPWM);
		break;
	case 22:
		EEP[m_imotor_ID - 1].usMaxPWM = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_PWM, &EEP[m_imotor_ID - 1].usMaxPWM);
		break;
	case 24:
		EEP[m_imotor_ID - 1].usOverloadPWMThreshold = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_OVERLOAD_PWM_THRESHOLD, &EEP[m_imotor_ID - 1].usOverloadPWMThreshold);
		break;
	case 26:
		EEP[m_imotor_ID - 1].usMinPosition = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_POSITION, &EEP[m_imotor_ID - 1].usMinPosition);
		break;
	case 28:
		EEP[m_imotor_ID - 1].usMaxPosition = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_POSITION, &EEP[m_imotor_ID - 1].usMaxPosition);
		break;
	case 30:
		EEP[m_imotor_ID - 1].usPositionKp = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KP, &EEP[m_imotor_ID - 1].usPositionKp);
		break;
	case 32:
		EEP[m_imotor_ID - 1].usPositionKd = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KD, &EEP[m_imotor_ID - 1].usPositionKd);
		break;
	case 34:
		EEP[m_imotor_ID - 1].usPositionKi = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KI, &EEP[m_imotor_ID - 1].usPositionKi);
		break;
	case 36:
		EEP[m_imotor_ID - 1].usPositionFeedforward1stGain = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_FEEDFORWARD_1ST_GAIN, &EEP[m_imotor_ID - 1].usPositionFeedforward1stGain);
		break;
	case 38:
		EEP[m_imotor_ID - 1].usPositionFeedforward2ndGain = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KP, &EEP[m_imotor_ID - 1].usPositionFeedforward2ndGain); //->usVelocityKp사용
		break;
	case 40:
		EEP[m_imotor_ID - 1].usVelocityKd = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KD, &EEP[m_imotor_ID - 1].usVelocityKd);
		break;
	case 42:
		EEP[m_imotor_ID - 1].usVelocityKi = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KI, &EEP[m_imotor_ID - 1].usVelocityKi);
		break;
	case 44:
		EEP[m_imotor_ID - 1].ucLEDBlinkPeriod = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_LED_BLINK_PERIOD, &EEP[m_imotor_ID - 1].ucLEDBlinkPeriod);
		break;
	case 45:
		EEP[m_imotor_ID - 1].ucADCFaultCheckPeriod = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ADC_FAULT_CHECK_PERIOD, &EEP[m_imotor_ID - 1].ucADCFaultCheckPeriod);
		break;
	case 46:
		EEP[m_imotor_ID - 1].ucPacketGarbageCheckPeriod = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_PACKET_GARBAGE_CHECK_PERIOD, &EEP[m_imotor_ID - 1].ucPacketGarbageCheckPeriod);
		break;
	case 47:
		EEP[m_imotor_ID - 1].ucStopDetectionPeriod = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_STOP_DETECTION_PERIOD, &EEP[m_imotor_ID - 1].ucStopDetectionPeriod);
		break;
	case 48:
		EEP[m_imotor_ID - 1].ucOverloadDetectionPeriod = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_OVELOAD_DETECTION_PERIOD, &EEP[m_imotor_ID - 1].ucOverloadDetectionPeriod);
		break;
	case 49:
		EEP[m_imotor_ID - 1].ucStopThreshold = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_STOP_THRESHOLD, &EEP[m_imotor_ID - 1].ucStopThreshold);
		break;
	case 50:
		EEP[m_imotor_ID - 1].ucInpositionMargin = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_INPOSITION_MARGIN, &EEP[m_imotor_ID - 1].ucInpositionMargin);
		break;
	case 51:
		EEP[m_imotor_ID - 1].ucReserved5 = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_RESERVED_5, &EEP[m_imotor_ID - 1].ucReserved5);
		break;
	case 52:
		EEP[m_imotor_ID - 1].cCalibrationDifference_L = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_RESERVED_6, &EEP[m_imotor_ID - 1].cCalibrationDifference_L);
		break;
	case 53:
		EEP[m_imotor_ID - 1].cCalibrationDifference_H = cData;
		nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_CALIBRATION_DIFFERENCE, &EEP[m_imotor_ID - 1].cCalibrationDifference_H);
		break;

	default:
		printf("This is a read-only address !\n");
		break;

	}

	ser.write(szSendBuffer, nPacketLength);
}


int CALLING_CONVENTION set_stat_cmd(void* buffer, unsigned char sid)
{
	DrsData* data = 0;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_STAT, 0);

	data = get_packet_data(buffer);
	data->stStatData.ucError = 0;
	data->stStatData.ucDetail = 0;
	return set_packet(buffer, sid, CMD_STAT, 0);
}

int CALLING_CONVENTION set_rollback_cmd(void* buffer, unsigned char sid, unsigned char id_skip, unsigned char baud_skip)
{
	DrsData* data = 0;
	if (buffer == 0)
		return set_packet(buffer, sid, CMD_ROLLBACK, sizeof(DrsRollbackData));

	data = get_packet_data(buffer);
	data->stRollbackData.ucIdSkip = id_skip;
	data->stRollbackData.ucBaudSkip = baud_skip;
	return set_packet(buffer, sid, CMD_ROLLBACK, sizeof(DrsRollbackData));
}

int CALLING_CONVENTION set_reboot_cmd(void* buffer, unsigned char sid)
{
	return set_packet(buffer, sid, CMD_REBOOT, 0);
}

int CALLING_CONVENTION set_calset_cmd(void* buffer, unsigned char sid)
{
	return set_packet(buffer, sid, CMD_CALSET, 0);
}

/**********************************************************************************************************************************/
//Call Back//
eep_write_ack* _eep_write_ack_callback = 0;
eep_map_write_ack* _eep_map_write_ack_callback = 0;
eep_read_ack* _eep_read_ack_callback = 0;
eep_map_read_ack* _eep_map_read_ack_callback = 0;

ram_write_ack* _ram_write_ack_callback = 0;
ram_write_ack* _ram_map_write_ack_callback = 0;
ram_read_ack* _ram_read_ack_callback = 0;
ram_map_read_ack* _ram_map_read_ack_callback = 0;

i_jog_ack* _i_jog_ack_callback = 0;
s_jog_ack* _s_jog_ack_callback = 0;
mi_jog_ack* _mi_jog_ack_callback = 0;
ms_jog_ack* _ms_jog_ack_callback = 0;
stat_ack* _stat_ack_callback = 0;
rollback_ack* _rollback_ack_callback = 0;
reboot_ack* _reboot_ack_callback = 0;


void  CALLING_CONVENTION regist_ack_callback_eep_write(eep_write_ack* callback)
{
	_eep_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_map_write(eep_map_write_ack* callback)
{
	_eep_map_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_read(eep_read_ack* callback)
{
	_eep_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_map_read(eep_map_read_ack* callback)
{
	_eep_map_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_write(ram_write_ack* callback)
{
	_ram_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_map_write(ram_map_write_ack* callback)
{
	_ram_map_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_read(ram_read_ack* callback)
{
	_ram_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_map_read(ram_map_read_ack* callback)
{
	_ram_map_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_i_jog(i_jog_ack* callback)
{
	_i_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_s_jog(s_jog_ack* callback)
{
	_s_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_mi_jog(mi_jog_ack* callback)
{
	_mi_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ms_jog(ms_jog_ack* callback)
{
	_ms_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_stat(stat_ack* callback)
{
	_stat_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_rollback(rollback_ack* callback)
{
	_rollback_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_reboot(reboot_ack* callback)
{
	_reboot_ack_callback = callback;
}


RAMRegisterMap _ramMap;
EEPRegisterMap _eepMap;


void proc_eep_read_ack(DrsPacket* packet)
{
	EEPRegisterMap _eepMap;

	if (_eep_read_ack_callback != 0)
		(*_eep_read_ack_callback)(
			packet->ucChipID,
			packet->unData.stRWData.ucAddress,
			packet->unData.stRWData.ucLen,
			packet->unData.stRWData.ucData,
			get_ack_status(packet)->ucError,
			get_ack_status(packet)->ucDetail
			);

	if (_eep_map_read_ack_callback != 0)
	{
		memcpy((void*)((char*)& _eepMap + packet->unData.stRWData.ucAddress), packet->unData.stRWData.ucData, packet->unData.stRWData.ucLen);
		(*_eep_map_read_ack_callback)(
			packet->ucChipID,
			packet->unData.stRWData.ucAddress,
			packet->unData.stRWData.ucLen,
			_eepMap,
			get_ack_status(packet)->ucError,
			get_ack_status(packet)->ucDetail
			);

	}
}

void proc_ram_read_ack(DrsPacket* packet)
{
	RAMRegisterMap _ramMap;

	if (_ram_read_ack_callback != 0)
		(*_ram_read_ack_callback)(
			packet->ucChipID,
			packet->unData.stRWData.ucAddress,
			packet->unData.stRWData.ucLen,
			packet->unData.stRWData.ucData,
			get_ack_status(packet)->ucError,
			get_ack_status(packet)->ucDetail
			);
	if (_ram_map_read_ack_callback != 0)
	{
		memcpy((void*)((char*)& _ramMap + packet->unData.stRWData.ucAddress), packet->unData.stRWData.ucData, packet->unData.stRWData.ucLen);
		(*_ram_map_read_ack_callback)(
			packet->ucChipID,
			packet->unData.stRWData.ucAddress,
			packet->unData.stRWData.ucLen,
			_ramMap,
			get_ack_status(packet)->ucError,
			get_ack_status(packet)->ucDetail
			);
	}
}


int CALLING_CONVENTION parse(void* buffer, int length, int* pos)
{
	DrsPacket* packet = get_ack_packet(buffer, length, pos);
	if (packet == 0)
		return 0;


	switch (packet->ucCmd)
	{
	case CMD_EEP_WRITE_ACK:
		if (_eep_write_ack_callback != 0)
			(*_eep_write_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_EEP_READ_ACK:
		proc_eep_read_ack(packet);
		break;
	case CMD_RAM_WRITE_ACK:
		if (_ram_write_ack_callback != 0)
			(*_ram_write_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_RAM_READ_ACK:
		proc_ram_read_ack(packet);
		break;
	case CMD_I_JOG_ACK:
		if (_i_jog_ack_callback != 0)
			(*_i_jog_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_S_JOG_ACK:
		if (_s_jog_ack_callback != 0)
			(*_s_jog_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_MI_JOG_ACK:
		if (_mi_jog_ack_callback != 0)
			(*_mi_jog_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_MS_JOG_ACK:
		if (_ms_jog_ack_callback != 0)
			(*_ms_jog_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_STAT_ACK:
		if (_stat_ack_callback != 0)
			(*_stat_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_ROLLBACK_ACK:
		if (_rollback_ack_callback != 0)
			(*_rollback_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	case CMD_REBOOT_ACK:
		if (_reboot_ack_callback != 0)
			(*_reboot_ack_callback)(
				packet->ucChipID,
				get_ack_status(packet)->ucError,
				get_ack_status(packet)->ucDetail
				);
		break;
	default:
		return 0;
	}

	return 1;
}
/**********************************************************************************************************************************/


 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //Move Command//
bool Herkulex_ErrorClear(unsigned char cID)
{
	bool bResult = false;

	// 에러 초기화
	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_STATUS_ERROR, &ucValue);
	ser.write(szSendBuffer, nPacketLength);

	// 에러 상세 초기화
	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_STATUS_DETAIL, &ucValue);
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool Herkulex_Servo_Enable(unsigned char cID, unsigned int uiMode)
{
	bool bResult = false;

	memset(szSendBuffer, 0, sizeof(szSendBuffer));

	switch (uiMode)
	{
	case 0:
		ucValue = TORQUE_CONTROL_FREE;
		nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
		ser.write(szSendBuffer, nPacketLength);
		bResult = true;
		break;
	case 1:
		ucValue = TORQUE_CONTROL_TORQUEON;
		nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
		ser.write(szSendBuffer, nPacketLength);
		bResult = true;
		break;
	case 2:
		ucValue = TORQUE_CONTROL_BRAKEON;
		nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
		ser.write(szSendBuffer, nPacketLength);
		bResult = true;
		break;

	}

	return bResult;
}

bool Position_Move(unsigned char cID, unsigned int iLED, unsigned int iPlayTime, int iTargetPos, int iJogMode/*int iprofile, bool bMulti, int turn*/)
{
	bool bResult = false;

	switch (iJogMode)
	{
	case 0: //S_JOG
		// S_JOG 값 초기화
		sjogs[cID - 1].Value = 0;
		sjogs[cID - 1].Stop = 0;
		sjogs[cID - 1].LED = iLED;
		sjogs[cID - 1].NoAction = 0;
		sjogs[cID - 1].ID = szIDs[cID - 1];
		sjogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
		sjogs[cID - 1].Profile = 0; //iprofile; //default:0

		//Target Command -> Count
		sjogs[cID - 1].Value = iTargetPos;

		memset(szSendBuffer, 0, sizeof(szSendBuffer));
		nPacketLength = set_s_jog_cmd(szSendBuffer, cID, iPlayTime, &sjogs[cID - 1], 1);
		break;
	case 1: //I_JOG
		// I_JOG 값 초기화
		ijogs[cID - 1].Value = 0;
		ijogs[cID - 1].Stop = 0;
		ijogs[cID - 1].LED = iLED;
		ijogs[cID - 1].NoAction = 0;
		ijogs[cID - 1].ID = szIDs[cID - 1];
		ijogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
		ijogs[cID - 1].Profile = 0; //iprofile; //default:0

		//Target Command -> Count
		ijogs[cID - 1].Value = iTargetPos;

		//Play time//
		ijogs[cID - 1].PlayTime_ms = iPlayTime;

		memset(szSendBuffer, 0, sizeof(szSendBuffer));
		nPacketLength = set_i_jog_cmd(szSendBuffer, cID, &ijogs[cID - 1], 1);
		break;
	// case 2: //MS_JOG
	// 	// MS_JOG 값 초기화
	// 	msjogs[cID - 1].Value = 0;
	// 	msjogs[cID - 1].Stop = 0;
	// 	msjogs[cID - 1].LED = iLED;
	// 	msjogs[cID - 1].NoAction = 0;
	// 	msjogs[cID - 1].ID = szIDs[cID - 1];
	// 	msjogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
	// 	msjogs[cID - 1].Profile = iprofile; //추가된 프로파일 모드

	// 	//Target Command -> Count
	// 	msjogs[cID - 1].Value = iTargetPos;

	// 	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	// 	if (bMulti)
	// 		nPacketLength = set_ms_jog_cmd(szSendBuffer, cID, iPlayTime, &msjogs[cID - 1], 1, turn, true);
	// 	else
	// 		nPacketLength = set_ms_jog_cmd(szSendBuffer, cID, iPlayTime, &msjogs[cID - 1], 1, turn, false);

	// 	break;
	// case 3: //MI_JOG
	// 	// MI_JOG 값 초기화
	// 	mijogs[cID - 1].Value = 0;
	// 	mijogs[cID - 1].Stop = 0;
	// 	mijogs[cID - 1].LED = iLED;
	// 	mijogs[cID - 1].NoAction = 0;
	// 	mijogs[cID - 1].ID = szIDs[cID - 1];
	// 	mijogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
	// 	mijogs[cID - 1].Profile = iprofile; //추가된 프로파일 모드

	// 	//Target Command -> Count
	// 	mijogs[cID - 1].Value = iTargetPos;
	// 	//Play time//
	// 	mijogs[cID - 1].PlayTime_ms = iPlayTime;

	// 	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	// 	if (bMulti)
	// 		nPacketLength = set_mi_jog_cmd(szSendBuffer, cID, &mijogs[cID - 1], 1, turn, true);
	// 	else
	// 		nPacketLength = set_mi_jog_cmd(szSendBuffer, cID, &mijogs[cID - 1], 1, turn, false);
	// 	break;
	}

	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool Velocity_Move(unsigned char cID, unsigned int iLED, int iTargetVel, int iJogMode/*,int iprofile, bool bMulti*/)
{
	bool bResult = false;

	switch (iJogMode)
	{
	case 0: //S_JOG
		// S_JOG 값 초기화
		sjogs[cID - 1].Value = 0;
		sjogs[cID - 1].Stop = 0;
		sjogs[cID - 1].LED = iLED;
		sjogs[cID - 1].NoAction = 0;
		sjogs[cID - 1].ID = szIDs[cID - 1];
		sjogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
		sjogs[cID - 1].Profile = 0; //iprofile; //default:0

		//Target Command -> Count
		sjogs[cID - 1].Value = iTargetVel;

		memset(szSendBuffer, 0, sizeof(szSendBuffer));
		nPacketLength = set_s_jog_cmd(szSendBuffer, cID, 10, &sjogs[cID - 1], 1);
		break;
	case 1: //I_JOG
		// I_JOG 값 초기화
		ijogs[cID - 1].Value = 0;
		ijogs[cID - 1].Stop = 0;
		ijogs[cID - 1].LED = iLED;
		ijogs[cID - 1].NoAction = 0;
		ijogs[cID - 1].ID = szIDs[cID - 1];
		ijogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
		ijogs[cID - 1].Profile = 0; //iprofile; //default:0

		//Target Command -> Count
		ijogs[cID - 1].Value = iTargetVel;

		memset(szSendBuffer, 0, sizeof(szSendBuffer));
		nPacketLength = set_i_jog_cmd(szSendBuffer, cID, &ijogs[cID - 1], 1);
		break;
	// case 2: //MS_JOG
	// 	// MS_JOG 값 초기화
	// 	msjogs[cID - 1].Value = 0;
	// 	msjogs[cID - 1].Stop = 0;
	// 	msjogs[cID - 1].LED = iLED;
	// 	msjogs[cID - 1].NoAction = 0;
	// 	msjogs[cID - 1].ID = szIDs[cID - 1];
	// 	msjogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
	// 	msjogs[cID - 1].Profile = iprofile; //추가된 프로파일 모드

	// 	//Target Command -> Count
	// 	msjogs[cID - 1].Value = iTargetVel;

	// 	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	// 	if (bMulti)
	// 		nPacketLength = set_ms_jog_cmd(szSendBuffer, cID, 10, &msjogs[cID - 1], 1, 0, true);
	// 	else
	// 		nPacketLength = set_ms_jog_cmd(szSendBuffer, cID, 10, &msjogs[cID - 1], 1, 0, false);

	// 	break;
	// case 3: //MI_JOG
	// 	// MI_JOG 값 초기화
	// 	mijogs[cID - 1].Value = 0;
	// 	mijogs[cID - 1].Stop = 0;
	// 	mijogs[cID - 1].LED = iLED;
	// 	mijogs[cID - 1].NoAction = 0;
	// 	mijogs[cID - 1].ID = szIDs[cID - 1];
	// 	mijogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
	// 	mijogs[cID - 1].Profile = iprofile; //추가된 프로파일 모드

	// 	//Target Command -> Count
	// 	mijogs[cID - 1].Value = iTargetVel;

	// 	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	// 	if (bMulti)
	// 		nPacketLength = set_mi_jog_cmd(szSendBuffer, cID, &mijogs[cID - 1], 1, 0, true);
	// 	else
	// 		nPacketLength = set_mi_jog_cmd(szSendBuffer, cID, &mijogs[cID - 1], 1, 0, false);

	// 	break;
	}
	

	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool S_JOG_MOVE(unsigned int iPlayTime, unsigned int iTotal_Axis, CMD_SJog * sjogArr)
{
	bool bResult = false;

	for (int i = 0; i < iTotal_Axis; i++)
	{
		sjogs[i].Stop = sjogArr[i].Stop;
		sjogs[i].LED = sjogArr[i].LED;
		sjogs[i].NoAction = sjogArr[i].NoAction;
		sjogs[i].ID = sjogArr[i].ID; //szIDs[i];
		sjogs[i].InfiniteTurn = sjogArr[i].InfiniteTurn;
		sjogs[i].Profile = sjogArr[i].Profile;
		//Target Command -> Count
		sjogs[i].Value = sjogArr[i].Value;
	}

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_s_jog_cmd(szSendBuffer, 0xFE, iPlayTime, sjogs, iTotal_Axis);

	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool I_JOG_MOVE(unsigned int iTotal_Axis, CMD_IJog * ijogArr)
{
	bool bResult = false;
	
	for (int i = 0; i < iTotal_Axis; i++)
	{
		ijogs[i].Stop = ijogArr[i].Stop;
		ijogs[i].LED = ijogArr[i].LED;
		ijogs[i].NoAction = ijogArr[i].NoAction;
		ijogs[i].ID = ijogArr[i].ID;
		ijogs[i].InfiniteTurn = ijogArr[i].InfiniteTurn;
		ijogs[i].Profile = ijogArr[i].Profile;
		//Target Command -> Count
		ijogs[i].Value = ijogArr[i].Value;
		//PlayTime_ms
		ijogs[i].PlayTime_ms = ijogArr[i].PlayTime_ms;
	}

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_i_jog_cmd(szSendBuffer, 0xFE, ijogs, iTotal_Axis);
	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool MS_JOG_MOVE(unsigned int iPlayTime, unsigned int iTotal_Axis, bool bMulti, int turn, CMD_MSJog * msjogArr)
{
	bool bResult = false;

	for (int i = 0; i < iTotal_Axis; i++)
	{
		msjogs[i].Stop = msjogArr[i].Stop;
		msjogs[i].LED = msjogArr[i].LED;
		msjogs[i].NoAction = msjogArr[i].NoAction;
		msjogs[i].ID = msjogArr[i].ID; //szIDs[i];
		msjogs[i].InfiniteTurn = msjogArr[i].InfiniteTurn;
		msjogs[i].Profile = msjogArr[i].Profile;
		//Target Command -> Count
		msjogs[i].Value = msjogArr[i].Value;
	}

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	if (bMulti)
		nPacketLength = set_ms_jog_cmd(szSendBuffer, 0xFE, iPlayTime, msjogs, 1, turn, true);
	else
		nPacketLength = set_ms_jog_cmd(szSendBuffer, 0xFE, iPlayTime, msjogs, 1, turn, false);

	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

bool MI_JOG_MOVE(unsigned int iTotal_Axis, bool bMulti, int turn, CMD_MIJog * mijogArr)
{
	bool bResult = false;

	for (int i = 0; i < iTotal_Axis; i++)
	{
		mijogs[i].Stop = mijogArr[i].Stop;
		mijogs[i].LED = mijogArr[i].LED;
		mijogs[i].NoAction = mijogArr[i].NoAction;
		mijogs[i].ID = mijogArr[i].ID;
		mijogs[i].InfiniteTurn = mijogArr[i].InfiniteTurn;
		mijogs[i].Profile = mijogArr[i].Profile;
		//Target Command -> Count
		mijogs[i].Value = mijogArr[i].Value;
		//PlayTime_ms
		mijogs[i].PlayTime_ms = mijogArr[i].PlayTime_ms;
	}

	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	if (bMulti)
		nPacketLength = set_mi_jog_cmd(szSendBuffer, 0xFE, mijogs, 1, turn, true);
	else
		nPacketLength = set_mi_jog_cmd(szSendBuffer, 0xFE, mijogs, 1, turn, false);

	//Send//
	ser.write(szSendBuffer, nPacketLength);

	bResult = true;
	return bResult;
}

/*********************************************************************************************************************************/
void FactoryReset(unsigned char cID, int iID_Skip, int iBaudrate_Skip)
{
	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_rollback_cmd(szSendBuffer, cID, iID_Skip, iBaudrate_Skip);
	//Send//
	ser.write(szSendBuffer, nPacketLength);

}

void Reboot(unsigned char cID)
{
	memset(szSendBuffer, 0, sizeof(szSendBuffer));
	nPacketLength = set_reboot_cmd(szSendBuffer, cID);
	//Send//
	ser.write(szSendBuffer, nPacketLength);

}
/*********************************************************************************************************************************/

// //Serial - Callback//
// void write_callback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO_STREAM("Writing to serial port" << msg->data);
//     ser.write(msg->data);
// }

//HerkuleX Command Service call//
bool Register_Command(HerkuleX::HerkuleX_RegisterCommand::Request  &req, 
					  HerkuleX::HerkuleX_RegisterCommand::Response &res)
{
	bool bResult = false;
	m_iModel = req.Model_Num; //Model Check
	//0:DRS-0101, 1:DRS-0102, 2:DRS-0201, 3:DRS-0301, 4:DRS-0302, 5:DRS-0303
	//6:DRS-0401, 7:DRS-0402, 8:DRS-0601, 9:DRS-0602 

	if(req.command == "RAM_RegisterData_Read_All")
	{
		res.command_Result = RAM_RegisterData_Read_ALL(req.ID, RAM_ID, RAM_LAST);
		m_bRAM_ReadAll_Flag = true;
	}
	else if(req.command == "EEP_RegisterData_Read_All")
	{
		res.command_Result = EEP_RegisterData_Read_ALL(req.ID, EEP_MODEL_NO_1, EEP_LAST);
		m_bEEP_ReadAll_Flag = true;
	}
	else if(req.command == "RAM_RegisterData_Read")
	{
		res.command_Result = RAM_RegisterData_Read(req.ID, req.Addr);
		m_bRAM_ReadAll_Flag = false;
	}
	else if(req.command == "EEP_RegisterData_Read")
	{
		res.command_Result = EEP_RegisterData_Read(req.ID, req.Addr);
		m_bEEP_ReadAll_Flag = false;
	}
	else if(req.command == "RAM_RegisterData_Write")
	{
		RAM_RegisterData_Write(req.ID, req.Addr, req.Value);
		res.command_Result = true;
	}
	else if(req.command == "EEP_RegisterData_Write")
	{
		EEP_RegisterData_Write(req.ID, req.Addr, req.Value);
		res.command_Result = true;
	}
	else if(req.command == "SERVO_ON")
	{
		res.command_Result = Herkulex_Servo_Enable(req.ID, 1);
		printf("SERVO_ON Call! \n");
	}
	else if(req.command == "SERVO_OFF")
	{
		res.command_Result = Herkulex_Servo_Enable(req.ID, 0);
		printf("SERVO_OFF Call! \n");
	}
	else if(req.command == "BRAKE_ON")
	{
		res.command_Result = Herkulex_Servo_Enable(req.ID, 2);
		printf("BRAKE_ON Call! \n");
	}
	else if(req.command == "ERROR_CLEAR")
	{
		res.command_Result = Herkulex_ErrorClear(req.ID);
		printf("ERROR_CLEAR Call! \n");
	}
	
	/*
	req.command
	req.ID
	req.Addr
	req.Value
	---
	res.command_Result
	*/
	bResult = true;
	res.command_Result = bResult;
	return bResult;
}

bool Position_Command(HerkuleX::HerkuleX_PositionMove::Request  &req, 
					  HerkuleX::HerkuleX_PositionMove::Response &res)
{
	bool bResult = false;

	//res.command_Result = Position_Move(req.ID, req.LED, req.PlayTime, req.TargetPosition, req.JogMode, req.Profile, req.Multi, req.Turn);
	res.command_Result = Position_Move(req.ID, req.LED, req.PlayTime, req.TargetPosition, req.JogMode);
	//printf("Position_Move Call ! \n");
	res.command = "Position_Move";
	/*
	req.ID
	req.LED
	req.PlayTime
	req.TargetPosition
	req.JogMode
	//req.Profile
	//req.Multi
	//req.Turn
	---
	res.command
	res.command_Result
	*/
	bResult = true;
	res.command_Result = bResult;
	return bResult;
}

bool Velocity_Command(HerkuleX::HerkuleX_VelocityMove::Request  &req, 
					  HerkuleX::HerkuleX_VelocityMove::Response &res)
{
	bool bResult = false;

	//res.command_Result = Velocity_Move(req.ID, req.LED, req.TargetVelocity, req.JogMode, req.Profile, req.Multi);
	res.command_Result = Velocity_Move(req.ID, req.LED, req.TargetVelocity, req.JogMode);
	//printf("Velocity_Move Call ! \n");
	res.command = "Velocity_Move";
	/*
	req.ID
	req.LED
	req.TargetVelocity
	req.JogMode
	//req.Profile
	//req.Multi
	---
	res.command
	res.command_Result
	*/
	bResult = true;
	res.command_Result = bResult;
	return bResult;
}

bool SJOG_Command(HerkuleX::HerkuleX_SJOG_Move::Request  &req, 
				  HerkuleX::HerkuleX_SJOG_Move::Response &res)
{
	bool bResult = false;

	for(int i=0; i<req.Total_Axis; i++)
	{
		sjogArr[i].Stop = 0;
		sjogArr[i].LED = req.LED_Arr[i];
		sjogArr[i].NoAction = 0;
		sjogArr[i].ID = req.ID_Arr[i];
		sjogArr[i].InfiniteTurn = 0x00; //Position Move
		sjogArr[i].Profile = 0;
		sjogArr[i].Value = req.TargetPosition_Arr[i];
	}

	res.command_Result = S_JOG_MOVE(req.PlayTime, req.Total_Axis, sjogArr);
	res.command = "SJOG_Move";
	/*
	int8[253]  ID_Arr
	int8[253]  LED_Arr
	int16[253] TargetPosition_Arr
	int16 PlayTime
	int16 Total_Axis

	---
	string command 
	bool command_Result
	*/
	bResult = true;
	res.command_Result = bResult;
	return bResult;
}

bool IJOG_Command(HerkuleX::HerkuleX_IJOG_Move::Request  &req, 
				  HerkuleX::HerkuleX_IJOG_Move::Response &res)
{
	bool bResult = false;

	for(int i=0; i<req.Total_Axis; i++)
	{
		ijogArr[i].Stop = 0;
		ijogArr[i].LED = req.LED_Arr[i];
		ijogArr[i].NoAction = 0;
		ijogArr[i].ID = req.ID_Arr[i];
		ijogArr[i].InfiniteTurn = 0x00; //Position Move
		ijogArr[i].Profile = 0;
		ijogArr[i].Value = req.TargetPosition_Arr[i];
		ijogArr[i].PlayTime_ms = req.PlayTime[i];

	}

	res.command_Result = I_JOG_MOVE(req.Total_Axis, ijogArr);
	res.command = "IJOG_Move";
	/*
	int8[253]  ID_Arr
	int8[253]  LED_Arr
	int16[253] TargetPosition_Arr
	int16[253] PlayTime
	int16 Total_Axis

	---
	string command 
	bool command_Result
	*/
	bResult = true;
	res.command_Result = bResult;
	return bResult;
}

bool HerkuleX_IDscan(int m_iCNT)
{
	bool bResult = false;
	m_bEEP_ReadAll_Flag = true;
	EEP_RegisterData_Read_ALL(m_iCNT, 0, EEP_LAST);

	bResult = true;
	return bResult;
}


void *t_function(void *data)
{
    int cmd = 0;
    int m_iID = 0;
	int m_iAddr = 0;
    int m_iValue = 0;
	int m_iScanCNT = 0;
	int m_iTotal_Axis = 0;

    while(1)
    {
        printf("******************************************\n");   
        printf("1:  Servo_On: (1,ID,0,1)\n");
        printf("    Servo_Off:(1,ID,0,0)\n");
        printf("    Brake On: (1,ID,0,2)\n");
        printf("2:  Velocity Move (2,ID,0,Value)\n");
        printf("3:  Position Move (3,ID,PlayTime,Value)\n");
        printf("4:  RAM_Read  (4,ID,Address,Model_Num)\n");
		printf("5:  EEP_Read  (5,ID,Address,Model_Num)\n");
		printf("6:  RAM_Write (6,ID,Address,Value)\n");
		printf("7:  EEP_Write (7,ID,Address,Value)\n");
		printf("8:  RAM_Read_All (8,ID,0,Model_Num)\n");
		printf("9:  EEP_Read_All (9,ID,0,Model_Num)\n");
		printf("10: Reboot (10,ID,0,0)\n");
		printf("11: FactoryReset (11,ID,0,0)\n");
		printf("12: ID Scan (12,0,0,0)\n");
		printf("13: Packet View Enable (13,0,0,0)\n");
		printf("14: Packet View Disable (14,0,0,)\n");
		printf("[==ex) Command Input: cmd, ID, Value_1, Value_2 ==]\n");
		printf("******************************************\n");
        scanf("%d,%d,%d,%d", &cmd, &m_iID, &m_iAddr, &m_iValue);
        
        switch(cmd)
        {
            case 0:
                break;
            case 1:
                Herkulex_Servo_Enable(m_iID, m_iValue);
		if(m_bView_Flag)
		{
			for(int i=0; i<nPacketLength; i++)
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
		}
                break;
            case 2:
                Velocity_Move(m_iID, 1, m_iValue, 0);
		if(m_bView_Flag)
		{
			for(int i=0; i<nPacketLength; i++)
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
		}
                break;
            case 3:
                Position_Move(m_iID, 1, m_iAddr, m_iValue, 0); //m_iAddr-> PlayTime으로 활용//
		if(m_bView_Flag)
		{
			for(int i=0; i<nPacketLength; i++)
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
		}
                break;
            case 4:
		m_iModel = m_iValue; //Model_Num
                RAM_RegisterData_Read(m_iID, m_iAddr); 
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
                break;
	    case 5:
		m_iModel = m_iValue; //Model_Num
		EEP_RegisterData_Read(m_iID, m_iAddr); 
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
                break;
	    case 6:
		RAM_RegisterData_Write(m_iID, m_iAddr, m_iValue);
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
		break;
	    case 7:
		EEP_RegisterData_Write(m_iID, m_iAddr, m_iValue);
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
		break;
	   case 8:
		m_iModel = m_iValue; //Model_Num
		m_bRAM_ReadAll_Flag = true;
		RAM_RegisterData_Read_ALL(m_iID, 0, RAM_LAST); 

		break;
	   case 9:
		m_iModel = m_iValue; //Model_Num
		m_bEEP_ReadAll_Flag = true;
		EEP_RegisterData_Read_ALL(m_iID, 0, EEP_LAST); 
		break;
	   case 10:
		Reboot(m_iID);
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
		break;
	   case 11:
		FactoryReset(m_iID, 1, 1);
		if(m_bView_Flag)
		{
			w_buffer.data.resize(nPacketLength);
			for(int i=0; i<nPacketLength; i++)
			{   
				ROS_INFO("Send_Packet[%d]:  %02x\n",i, szSendBuffer[i]);
				w_buffer.data[i] = szSendBuffer[i];
			}
			write_pub.publish(w_buffer);
		}
		break;
	   case 12:
		HerkuleX_IDscan(253);
		usleep(30000);
		for(int i=0; i<253; i++)
		{
			HerkuleX_IDscan(i+1);
			usleep(30000);
			printf("[Check_%d]: ID = %d \n",i+1, EEP[i+1].ucID);
			printf("[Check_%d]: Model = DRS-0%d0%d \n",i+1, EEP[i+1].ucModelNo1, EEP[i+1].ucModelNo2);
			if(EEP[i+1].ucID != 0)
			{
				m_iTotal_Axis++;
			}
		}
		printf("###[Total Axis]: %d ###\n",m_iTotal_Axis);
		m_iTotal_Axis = 0; //Reset count//
		break;
	   case 13:
		m_bView_Flag = true;
		break;
	   case 14:
		m_bView_Flag = false;
		break;
	   default:
		printf("[Error]: Not defined Command... \n");
		break;
        }
        usleep(100);
    }

    pthread_cancel(p_thread); //Thread kill
}

//int main (int argc, char** argv)
int main (int argc, char* argv[])
{
    	ros::init(argc, argv, "HerkuleX_node");
        ros::NodeHandle nh;
	ros::NodeHandle wh;
	//ros::NodeHandle rh;
	//ros::NodeHandle eh;
	ros::NodeHandle Rcmd_h;
	ros::NodeHandle Pcmd_h;
	ros::NodeHandle Vcmd_h;
	ros::NodeHandle SJOGcmd_h;
	ros::NodeHandle IJOGcmd_h;
	ros::NodeHandle R_array_h;
	ros::NodeHandle E_array_h;

	//Check number of connected HerkuleX//
	m_iTotal_Axis = atoi(argv[1]);
	ROS_INFO("Connected HerkuleX Total_Axis: %d", m_iTotal_Axis);
	
	//Serial-Publish
        read_pub = nh.advertise<std_msgs::UInt8MultiArray>("read", 1000);
	write_pub = wh.advertise<std_msgs::UInt8MultiArray>("write", 1000);

	/***************************************************************************************/
	//RAM_Info & EEP_Info Publish...add loop...
	string str_ram_pub;
	string str_eep_pub;
	for(int i=0; i<m_iTotal_Axis; i++)
	{
		str_ram_pub = "Info_RAM_ID_" + to_string(i+1);
		RAM_Array_pub[i+1] = R_array_h.advertise<HerkuleX::HerkuleX_Info_RAM>(str_ram_pub, 1000);

		str_eep_pub = "Info_EEP_ID_" + to_string(i+1);
		EEP_Array_pub[i+1] = E_array_h.advertise<HerkuleX::HerkuleX_Info_EEP>(str_eep_pub, 1000);

	}

	//Command Service
	Register_service = Rcmd_h.advertiseService("Register_cmd", Register_Command);
	PositionCmd_service = Pcmd_h.advertiseService("Position_cmd", Position_Command);
	VelocityCmd_service = Vcmd_h.advertiseService("Velocity_cmd", Velocity_Command);
	SJOGCmd_service = SJOGcmd_h.advertiseService("SJOG_cmd", SJOG_Command);
	IJOGCmd_service = IJOGcmd_h.advertiseService("IJOG_cmd", IJOG_Command);
	
	//init_memset//
	memset(RAM, 0, sizeof(RAM));
	memset(EEP, 0, sizeof(EEP));	
	memset(sjogs, 0, sizeof(sjogs));
	memset(ijogs, 0, sizeof(ijogs));
	memset(msjogs, 0, sizeof(msjogs));
	memset(mijogs, 0, sizeof(mijogs));
	memset(sjogArr, 0, sizeof(sjogArr));
	memset(ijogArr, 0, sizeof(ijogArr));

	////Callback init/////////
	regist_ack_callback_eep_write(EEPWriteAckCallback);
	regist_ack_callback_eep_read(EEPReadAckCallback);
	regist_ack_callback_ram_write(RAMWriteAckCallback);
	regist_ack_callback_ram_read(RAMReadAckCallback);
	regist_ack_callback_ram_map_read(RAMMapReadAckCallback);
	regist_ack_callback_eep_map_read(EEPMapReadAckCallback);
	regist_ack_callback_i_jog(IJogAckCallback);
	regist_ack_callback_s_jog(SJogAckCallback);
	regist_ack_callback_mi_jog(MIJogAckCallback);
	regist_ack_callback_ms_jog(MSJogAckCallback);
	regist_ack_callback_stat(StatAckCallback);
	regist_ack_callback_rollback(RollbackAckCallback);
	regist_ack_callback_reboot(RebootAckCallback);

    int thr_id;
    int status;
    int a = 1;

    thr_id = pthread_create(&p_thread, NULL, t_function, (void *)&a);
    if (thr_id < 0)
    {
        printf("thread create error !!");
        exit(0);
    }

    try
    {
        ser.setPort("/dev/HerkuleX"); //USB rules setting...
        ser.setBaudrate(115200); //default baudrate
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
	ROS_INFO_STREAM("HerkuleX Node Start !");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100); //100 Hz
    std_msgs::String DataTemp;
    int m_idata_size = 0;
    std_msgs::String result;
    std_msgs::String msg;

    while(ros::ok())
    {
        ros::spinOnce();
        if(ser.available())
        {
	    std_msgs::UInt8MultiArray  serial_data;
	    m_idata_size = ser.available();
            ser.read (serial_data.data, m_idata_size);
			if(m_bView_Flag)
            ROS_INFO("serial data size: %d ", m_idata_size);

            r_buffer.data.resize(m_idata_size);
            for(int i=0; i<m_idata_size; i++)
            {
		r_buffer.data[i] = serial_data.data[i];
		if(m_bView_Flag)
			ROS_INFO("Read[%d]: %02x", i, r_buffer.data[i]);

            }

		int pos = 0;
		unsigned char* buffer = new unsigned char[m_idata_size];
		std::copy(r_buffer.data.begin(), r_buffer.data.end(), buffer);
		parse(buffer, m_idata_size, &pos);

		delete[ ] buffer;

		read_pub.publish(r_buffer);
        }

        loop_rate.sleep();

    }

    return 0;
}

