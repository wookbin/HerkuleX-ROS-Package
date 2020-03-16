/*
HerkuleX_Single feedback Data All_Test_node
Model --> DRS-0101
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <math.h>
#include <unistd.h>

#include "HerkuleX_node.h"
#include "HerkuleX/MsgHerkuleX_RAM.h" //MSG
#include "HerkuleX/MsgHerkuleX_EEP.h" //MSG
#include "HerkuleX/HerkuleX_Info_RAM.h" //MSG_Array
#include "HerkuleX/HerkuleX_Info_EEP.h" //MSG_Array
#include "HerkuleX/HerkuleX_RegisterCommand.h" //SRV

using namespace std;
#include <pthread.h>

HerkuleX::MsgHerkuleX_RAM RAM_msg;
HerkuleX::MsgHerkuleX_EEP EEP_msg;
HerkuleX::HerkuleX_Info_RAM RAM_Array;
HerkuleX::HerkuleX_Info_EEP EEP_Array;
/*Subscriber************************/
ros::Subscriber RAM_Array_sub;
ros::Subscriber EEP_Array_sub;
/*Comand srv _ Client************************/
HerkuleX::HerkuleX_RegisterCommand Register_cmd;
ros::ServiceClient Register_client;
/*****************************/


pthread_t p_thread;
int cmd = 0;
int m_iID = 1; //DRS-0101 , ID: 1


void *t_function(void *data)
{

    while(1)
    {
        printf("******************************************\n");   
        printf("[0] RAM Register Read \n");
        printf("[1] EEP Register Read \n");
        printf("ex) 0 or 1, ID \n");
		printf("******************************************\n");
        scanf("%d,%d", &cmd, &m_iID );

        usleep(10000);
    }

    pthread_cancel(p_thread); //Thread kill
}

void subRAM_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];

        ROS_INFO("[RAM_msg]--------------------");
        ROS_INFO("ID: %d", RAM_msg.ID);
        ROS_INFO("AckPolicy: %d", RAM_msg.AckPolicy);
        ROS_INFO("AlarmLEDPolicy: %d", RAM_msg.AlarmLEDPolicy);
        ROS_INFO("TorquePolicy: %d", RAM_msg.TorquePolicy);
        ROS_INFO("MaxTemperature: %d", RAM_msg.MaxTemperature);
        ROS_INFO("MinVoltage: %d", RAM_msg.MinVoltage);
        ROS_INFO("MaxVoltage: %d", RAM_msg.MaxVoltage);
        ROS_INFO("AccelerationRatio: %d", RAM_msg.AccelerationRatio);
        ROS_INFO("MaxAccelerationTime: %d", RAM_msg.MaxAccelerationTime);
        ROS_INFO("DeadZone: %d", RAM_msg.DeadZone);
        ROS_INFO("SaturatorOffset: %d", RAM_msg.SaturatorOffset);
        ROS_INFO("SaturatorSlope: %d", RAM_msg.SaturatorSlope);
        ROS_INFO("PWMOffset: %d", RAM_msg.PWMOffset);
        ROS_INFO("MinPWM: %d", RAM_msg.MinPWM);
        ROS_INFO("MaxPWM: %d", RAM_msg.MaxPWM);
        ROS_INFO("OverloadPWMThreshold: %d", RAM_msg.OverloadPWMThreshold);
        ROS_INFO("MinPosition: %d", RAM_msg.MinPosition);
        ROS_INFO("MaxPosition: %d", RAM_msg.MaxPosition);
        ROS_INFO("PositionKp: %d", RAM_msg.PositionKp);
        ROS_INFO("PositionKd: %d", RAM_msg.PositionKd);
        ROS_INFO("PositionKi: %d", RAM_msg.PositionKi);
        ROS_INFO("PositionFeedforward1stGain: %d", RAM_msg.PositionFeedforward1stGain);
        ROS_INFO("PositionFeedforward2ndGain: %d", RAM_msg.PositionFeedforward2ndGain);
        ROS_INFO("LEDBlinkPeriod: %d", RAM_msg.LEDBlinkPeriod);
        ROS_INFO("ADCFaultCheckPeriod: %d", RAM_msg.ADCFaultCheckPeriod);
        ROS_INFO("PacketGarbageCheckPeriod: %d", RAM_msg.PacketGarbageCheckPeriod);
        ROS_INFO("StopDetectionPeriod: %d", RAM_msg.StopDetectionPeriod);
        ROS_INFO("OverloadDetectionPeriod: %d", RAM_msg.OverloadDetectionPeriod);
        ROS_INFO("StopThreshold: %d", RAM_msg.StopThreshold);
        ROS_INFO("InpositionMargin: %d", RAM_msg.InpositionMargin);
        ROS_INFO("CalibrationDifference_L: %d", RAM_msg.CalibrationDifference_L);
        ROS_INFO("CalibrationDifference_H: %d", RAM_msg.CalibrationDifference_H);
        ROS_INFO("StatusError: %d", RAM_msg.StatusError);
        ROS_INFO("StatusDetail: %d", RAM_msg.StatusDetail);
        ROS_INFO("TorqueControl: %d", RAM_msg.TorqueControl);
        ROS_INFO("LEDControl: %d", RAM_msg.LEDControl);
        ROS_INFO("Voltage: %d", RAM_msg.Voltage);
        ROS_INFO("Temperature: %d", RAM_msg.Temperature);
        ROS_INFO("CurrentControlMode: %d", RAM_msg.CurrentControlMode);
        ROS_INFO("Tick: %d", RAM_msg.Tick);
        ROS_INFO("CalibratedPosition: %d", RAM_msg.CalibratedPosition);
        ROS_INFO("AbsolutePosition: %d", RAM_msg.AbsolutePosition);
        ROS_INFO("DifferentialPosition: %d", RAM_msg.DifferentialPosition);
        ROS_INFO("PWM: %d", RAM_msg.PWM);
        ROS_INFO("AbsoluteGoalPosition: %d", RAM_msg.AbsoluteGoalPosition);
        ROS_INFO("AbsoluteDesiredTrajectoryPosition: %d", RAM_msg.AbsoluteDesiredTrajectoryPosition);
        ROS_INFO("DesiredVelocity: %d", RAM_msg.DesiredVelocity);
    }
    //int iIndex = msg->pHerkuleX_RAM.size();


}

void subEEP_Info(const HerkuleX::HerkuleX_Info_EEP:: ConstPtr& msg)
{
    //ROS_INFO("EEP_Info_msg size: %d",msg->pHerkuleX_EEP.size());
    for (int i = 0; i < msg->pHerkuleX_EEP.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_EEP &EEP_msg = msg->pHerkuleX_EEP[i];

        ROS_INFO("[EEP_msg]--------------------");
        ROS_INFO("ModelNo1: %d", EEP_msg.ModelNo1);
        ROS_INFO("ModelNo2: %d", EEP_msg.ModelNo2);
        ROS_INFO("Version1: %d", EEP_msg.Version1);
        ROS_INFO("Version2: %d", EEP_msg.Version2);
        ROS_INFO("BaudRate: %d", EEP_msg.BaudRate);
        ROS_INFO("ID: %d", EEP_msg.ID);
        ROS_INFO("AckPolicy: %d", EEP_msg.AckPolicy);
        ROS_INFO("AlarmLEDPolicy: %d", EEP_msg.AlarmLEDPolicy);
        ROS_INFO("TorquePolicy: %d", EEP_msg.TorquePolicy);
        ROS_INFO("MaxTemperature: %d", EEP_msg.MaxTemperature);
        ROS_INFO("MinVoltage: %d", EEP_msg.MinVoltage);
        ROS_INFO("MaxVoltage: %d", EEP_msg.MaxVoltage);
        ROS_INFO("AccelerationRatio: %d", EEP_msg.AccelerationRatio);
        ROS_INFO("MaxAccelerationTime: %d", EEP_msg.MaxAccelerationTime);
        ROS_INFO("DeadZone: %d", EEP_msg.DeadZone);
        ROS_INFO("SaturatorOffset: %d", EEP_msg.SaturatorOffset);
        ROS_INFO("SaturatorSlope: %d", EEP_msg.SaturatorSlope);
        ROS_INFO("PWMOffset: %d", EEP_msg.PWMOffset);
        ROS_INFO("MinPWM: %d", EEP_msg.MinPWM);
        ROS_INFO("MaxPWM: %d", EEP_msg.MaxPWM);
        ROS_INFO("OverloadPWMThreshold: %d", EEP_msg.OverloadPWMThreshold);
        ROS_INFO("MinPosition: %d", EEP_msg.MinPosition);
        ROS_INFO("MaxPosition: %d", EEP_msg.MaxPosition);
        ROS_INFO("PositionKp: %d", EEP_msg.PositionKp);
        ROS_INFO("PositionKd: %d", EEP_msg.PositionKd);
        ROS_INFO("PositionKi: %d", EEP_msg.PositionKi);
        ROS_INFO("PositionFeedforward1stGain: %d", EEP_msg.PositionFeedforward1stGain);
        ROS_INFO("PositionFeedforward2ndGain: %d", EEP_msg.PositionFeedforward2ndGain);
        ROS_INFO("LEDBlinkPeriod: %d", EEP_msg.LEDBlinkPeriod);
        ROS_INFO("ADCFaultCheckPeriod: %d", EEP_msg.ADCFaultCheckPeriod);
        ROS_INFO("PacketGarbageCheckPeriod: %d", EEP_msg.PacketGarbageCheckPeriod);
        ROS_INFO("StopDetectionPeriod: %d", EEP_msg.StopDetectionPeriod);
        ROS_INFO("OverloadDetectionPeriod: %d", EEP_msg.OverloadDetectionPeriod);
        ROS_INFO("StopThreshold: %d", EEP_msg.StopThreshold);
        ROS_INFO("InpositionMargin: %d", EEP_msg.InpositionMargin);
        ROS_INFO("CalibrationDifference_L: %d", EEP_msg.CalibrationDifference_L);
        ROS_INFO("CalibrationDifference_H: %d", EEP_msg.CalibrationDifference_H);
    }
  
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "HerkuleX_feedback_Test_node");
	ros::NodeHandle Rcmd_h;
    ros::NodeHandle Rsub_h;
    ros::NodeHandle Esub_h;

	//Command ServiceClient/////////////////////////////////////////////////////////////////////
    Register_client	 = Rcmd_h.serviceClient<HerkuleX::HerkuleX_RegisterCommand>("Register_cmd");

    //Subscriber/////////////////////////////////////////////////////
    RAM_Array_sub = Rsub_h.subscribe("Info_RAM_ID_1", 1000, subRAM_Info);
    EEP_Array_sub = Esub_h.subscribe("Info_EEP_ID_1", 1000, subEEP_Info);

    //thread/////////////////////////////////////////////////////////
    int thr_id;
    int a = 1;
    thr_id = pthread_create(&p_thread, NULL, t_function, (void *)&a);
    if (thr_id < 0)
    {
        printf("thread1 create error !!");
        exit(0);
    }
    /////////////////////////////////////////////////////////////////

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(100.0); //HZ

    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        if(cmd == 0)
        {
            Register_cmd.request.command = "RAM_RegisterData_Read_All";
            Register_cmd.request.Model_Num = 0; //DRS-0101
            Register_cmd.request.ID = m_iID;
            Register_cmd.request.Addr = RAM_ID; 
            Register_cmd.request.Value = RAM_LAST;
            Register_client.call(Register_cmd);

        }
        else
        {
            Register_cmd.request.command = "EEP_RegisterData_Read_All";
            Register_cmd.request.Model_Num = 0; //DRS-0101
            Register_cmd.request.ID = m_iID;
            Register_cmd.request.Addr = EEP_MODEL_NO_1; 
            Register_cmd.request.Value = EEP_LAST;
            Register_client.call(Register_cmd);
        }
    
        last_time = current_time;
        //ROS_INFO("Time: %.2f\n",dt);
        loop_rate.sleep();
    }

    return 0;
}

