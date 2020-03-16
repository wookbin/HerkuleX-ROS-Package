/*
HerkuleX_12DOF_Robot_node
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <serial/serial.h> //Bluetooth to USB

#include "HerkuleX_node.h"
#include "HerkuleX/MsgHerkuleX_RAM.h" //MSG
#include "HerkuleX/MsgHerkuleX_EEP.h" //MSG
#include "HerkuleX/HerkuleX_RegisterCommand.h" //SRV
#include "HerkuleX/HerkuleX_PositionMove.h" //SRV
#include "HerkuleX/HerkuleX_VelocityMove.h" //SRV
#include "HerkuleX/HerkuleX_SJOG_Move.h" //SRV
#include "HerkuleX/HerkuleX_IJOG_Move.h" //SRV

using namespace std;
#include <pthread.h>
#define BUF_LEN 4096

/*Coomand srv _ Client************************/
HerkuleX::HerkuleX_RegisterCommand Register_cmd[12];
HerkuleX::HerkuleX_PositionMove Position_cmd[12];
HerkuleX::HerkuleX_VelocityMove Velocity_cmd[12];
HerkuleX::HerkuleX_SJOG_Move SJOG_cmd;
HerkuleX::HerkuleX_IJOG_Move IJOG_cmd;

ros::ServiceClient Register_client;
ros::ServiceClient PositionCmd_client;
ros::ServiceClient VelocityCmd_client;
ros::ServiceClient SJOGCmd_client;
ros::ServiceClient IJOGCmd_client;
/********************************************/
serial::Serial ser;
std_msgs::String DataTemp;
std_msgs::String result;
string strTemp;
string s1, s2, s3, s4;
int m_idata_size = 0;

int m_iLinearVelocity = 0;
int m_iAngulerVelocity = 0;

/********************************************/
//Step Table
double m_dHome[12] = {10.0,-10.0,10.0,-10.0,0.0,0.0,0.0,0.0,30.0,30.0,30.0,30.0};

double m_dFrontMove_Step1[12] = {30.0,-10.0,10.0,-30.0,0.0,0.0,0.0,0.0,40.0,30.0,30.0,40.0};
double m_dFrontMove_Step2[12] = {45.0,-25.0,25.0,-45.0,0.0,0.0,0.0,0.0,55.0,30.0,30.0,55.0};
double m_dFrontMove_Step3[12] = {0.0,-25.0,25.0,0.0,0.0,0.0,0.0,0.0,55.0,30.0,30.0,55.0};
double m_dFrontMove_Step4[12] = {0.0,-40.0,40.0,0.0,0.0,0.0,0.0,0.0,30.0,40.0,40.0,30.0};
double m_dFrontMove_Step5[12] = {10.0,-40.0,40.0,-10.0,0.0,0.0,0.0,0.0,30.0,55.0,55.0,30.0};
double m_dFrontMove_Step6[12] = {10.0,0.0,0.0,-10.0,0.0,0.0,0.0,0.0,30.0,55.0,55.0,30.0};

double m_dTurnRight_Step1[12] = {25.0,-10.0,10.0,-25.0,15.0,-15.0,15.0,-15.0,70.0,30.0,30.0,70.0};
double m_dTurnRight_Step2[12] = {10.0,-25.0,25.0,-10.0,15.0,-15.0,15.0,-15.0,30.0,70.0,70.0,30.0};
double m_dTurnRight_Step3[12] = {10.0,-25.0,25.0,-10.0,0.0,-15.0,15.0,0.0,30.0,70.0,70.0,30.0};
double m_dTurnRight_Step4[12] = {25.0,-10.0,10.0,-25.0,0.0,-15.0,15.0,0.0,70.0,30.0,30.0,70.0};

double m_dTurnLeft_Step1[12] = {10.0,-25.0,25.0,-10.0,15.0,-15.0,15.0,-15.0,30.0,70.0,70.0,30.0};
double m_dTurnLeft_Step2[12] = {25.0,-10.0,10.0,-25.0,15.0,-15.0,15.0,-15.0,70.0,30.0,30.0,70.0};
double m_dTurnLeft_Step3[12] = {25.0,-10.0,10.0,-25.0,15.0,0.0,0.0,-15.0,70.0,30.0,30.0,70.0};
double m_dTurnLeft_Step4[12] = {10.0,-25.0,25.0,-10.0,15.0,0.0,0.0,-15.0,30.0,70.0,70.0,30.0};


pthread_t p_thread;
int cmd = 0;
int Value = 0;
bool m_Stop = false;

void *t_function(void *data)
{

    while(1)
    {
        /*
        printf("******************************************\n");   
        printf("[1]  Front Move \n");
        printf("[9]  STOP \n");
        printf("[10] Turn Right \n");
        printf("[30] Turn Left \n");
        printf("[20] Rear Move \n");
        
		printf("******************************************\n");
        scanf("%d,%d", &cmd, &Value);

        if(cmd == 9)
            m_Stop = true;
        else
            m_Stop = false;
        */

        
        if(ser.available() > 0)
        {
            DataTemp.data = ser.read();
            if(DataTemp.data == ".")
            {
                //ROS_INFO_STREAM("DATA: " << result.data);
                //int ilength = result.data.size();
                int iCommaPoint = result.data.find(','); 
                //ROS_INFO_STREAM("iCommaPoint: " << iCommaPoint);
                m_iLinearVelocity = stof(result.data.substr(0, iCommaPoint));
                m_iAngulerVelocity = stof(result.data.substr(iCommaPoint + 1));
                //ROS_INFO_STREAM("m_iLinearVelocity: " << m_iLinearVelocity);
                //ROS_INFO_STREAM("m_iAngulerVelocity: " << m_iAngulerVelocity);
                memset(&result.data, 0, sizeof(result.data));  
                usleep(1000);

            }
            else
            {
                result.data += DataTemp.data;
                //ROS_INFO_STREAM("Read: " << DataTemp.data);
            }
            
        }
        
        if(m_iLinearVelocity > 1)
        {
            cmd = 1;
            m_Stop = false;
        }
        else
        {
            if(m_iLinearVelocity == 0)
            {
                cmd = 9;
                m_Stop = true;
            }
            else
            {
                cmd = 20;
                m_Stop = false;
            }
            
        }

        // if(m_iAngulerVelocity > 1)
        // {
        //     cmd = 10;
        //     m_Stop = false;
        // }
        // else
        // {
        //     if(m_iAngulerVelocity == 0)
        //     {
        //         cmd = 9;
        //         m_Stop = true;
        //     }
        //     else
        //     {
        //         cmd = 30;
        //         m_Stop = false;
        //     }
            
        // }

        usleep(1000); //1ms
    }

    pthread_cancel(p_thread); //Thread kill
}

int Set_AngleToCount(double m_dAngle)
{
    int iResult = 0;

    iResult = (int)(m_dAngle/0.325) +512; //DRS-0101의 경우, 소수점은 버림
    // Count to Angle 시에는 (ucount - 512) * 0.325//

    return iResult;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "HerkuleX_12DOF_Robot_node");
	ros::NodeHandle Rcmd_h;
	ros::NodeHandle Pcmd_h;
	ros::NodeHandle Vcmd_h;
    ros::NodeHandle SJOGcmd_h;
    ros::NodeHandle IJOGcmd_h;
	
    //Command ServiceClient  
    Register_client	   = Rcmd_h.serviceClient<HerkuleX::HerkuleX_RegisterCommand>("Register_cmd");
	PositionCmd_client = Pcmd_h.serviceClient<HerkuleX::HerkuleX_PositionMove>("Position_cmd");
	VelocityCmd_client = Vcmd_h.serviceClient<HerkuleX::HerkuleX_VelocityMove>("Velocity_cmd");
    SJOGCmd_client = SJOGcmd_h.serviceClient<HerkuleX::HerkuleX_SJOG_Move>("SJOG_cmd");
    IJOGCmd_client = IJOGcmd_h.serviceClient<HerkuleX::HerkuleX_IJOG_Move>("IJOG_cmd");

    //thread/////////////////////////////////////////////////////////
    int thr_id;
    int a = 1;
    thr_id = pthread_create(&p_thread, NULL, t_function, (void *)&a);
    if (thr_id < 0)
    {
        printf("thread1 create error !!");
        exit(0);
    }

    //Initialize////////////////////////////////////
	//memset(sjogArr, 0, sizeof(sjogArr));

    for(int i=0; i<12; i++)
    {
        Register_cmd[i].request.command = "SERVO_ON";
        Register_cmd[i].request.ID = i+1;
        Register_client.call(Register_cmd[i]);
        usleep(10000); //10ms

    }

    for(int j=0; j<12; j++)
    {
        Position_cmd[j].request.ID = j+1;
        Position_cmd[j].request.LED = 5; //Yellow
        Position_cmd[j].request.PlayTime = 60; //Play Time(ms): 60.

        switch(j)
        {
            case 0:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//542; // 10 deg
                break;
            case 1:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//482; // -10 deg
                break;
            case 2:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//542; // 10 deg
                break;
            case 3:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//482; //-10 deg
                break;
            case 4:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//512; //0 deg
                break;
            case 5:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//512; //0 deg
                break;
            case 6:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//512; //0 deg
                break;
            case 7:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//512; //0 deg
                break;
            case 8:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//604; //30 deg
                break;
            case 9:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//604; //30 deg
                break;
            case 10:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//604; //30 deg
                break;
            case 11:
                Position_cmd[j].request.TargetPosition = Set_AngleToCount(m_dHome[j]);//604; //30 deg
                break;
        }

        Position_cmd[j].request.JogMode = 0;

        PositionCmd_client.call(Position_cmd[j]);

    }

    sleep(1);

    //bluetooth to USB Connet Loop /////////////////////////////////////////////
    try
    {
        //Bluetooth to USB_serial 
        ser.setPort("/dev/SENA");
        ser.setBaudrate(9600);
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
    }
    else
    {
        return -1;
    }
    ///////////////////////////////////////////////////////////////////////////
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(100.0); //HZ

    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        switch(cmd)
        {
            case 0:

                break;
            case 1:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step1[0]);//602;//30deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step1[1]);//482; //-10deg                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step1[2]);//542; //10 deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step1[3]);//419;//432;// -30deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step1[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step1[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step1[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step1[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step1[8]);//635; //40deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step1[9]);//604; //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step1[10]);//604; //30deg
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step1[11]);//635;//40 deg
           
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 2;
                }
                break;
            case 2:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step2[0]);//650; // 45deg //647;//
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step2[1]);//435; //-25deg //437;//
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step2[2]);//588; // 25deg //587;//
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step2[3]);//373;// -45deg//387;//

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step2[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step2[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step2[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step2[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step2[8]);//681; //55 deg //679;//

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step2[9]);//604; //30 deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step2[10]);//604; //30deg

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step2[11]);//681; // 55deg //679;//
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 3;
                }
                break;
            case 3:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step3[0]);//512;// 0deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step3[1]);//435; //-25 deg //437;
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step3[2]);//588; //25 deg //587;
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step3[3]);//512;// 0deg

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step3[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step3[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step3[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step3[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 5;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step3[8]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step3[9]);//604; //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step3[10]);//604; //30deg

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step3[11]);//681; //55deg //679;
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 4;
                }
                break;
            case 4:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step4[0]);//512; //0deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step4[1]);//388; // -40deg //392;//
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step4[2]);//635; //40deg //632;//
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 5;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step4[3]);//512; //0deg

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step4[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step4[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step4[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step4[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step4[8]);//604;// 30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 1;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step4[9]);//635; //40deg //634;//

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 1;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step4[10]);//635; // 40deg //634;//

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step4[11]);//604;// 30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 5;
                }
                break;
            case 5:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step5[0]);//558; //10 deg //557;//
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step5[1]);//388; //-40 deg //392;
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step5[2]);//635; //40deg //632;
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step5[3]);//481; //-10deg //467;//

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step5[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step5[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step5[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step5[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step5[8]);//604;//30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step5[9]);//681; // 55deg //679;

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 1;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step5[10]);//681; // 55deg //679;//

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step5[11]);//604; //30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 6;
                }
                break;
            case 6:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step6[0]);//588;// 10deg //557;
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step6[1]);//512;//0deg
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step6[2]);//512;//0deg
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 5;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step6[3]);//481; // -10deg //467;

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step6[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step6[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step6[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step6[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 5;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step6[8]);//604; //30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step6[9]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step6[10]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step6[11]);//604; //30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 1;
                }
                break;
            case 9:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dHome[0]);
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dHome[1]);                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dHome[2]);
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 5;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dHome[3]);
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dHome[4]);

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dHome[5]);

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dHome[6]);
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dHome[7]);

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 5;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dHome[8]);

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dHome[9]);

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dHome[10]);
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dHome[11]);
           
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                cmd = 0;
                break;
            case 10:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnRight_Step1[0]);//25deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnRight_Step1[1]); //-10deg               
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnRight_Step1[2]); //10deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnRight_Step1[3]);//-25deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnRight_Step1[4]); //15deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnRight_Step1[5]); //-15deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnRight_Step1[6]); //15deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnRight_Step1[7]); //-15deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnRight_Step1[8]); //70deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnRight_Step1[9]); //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnRight_Step1[10]); //30deg
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnRight_Step1[11]); //70deg
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 11;
                }
                break;
            case 11:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnRight_Step2[0]); //10deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnRight_Step2[1]); //-25deg                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnRight_Step2[2]); //25deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnRight_Step2[3]); //-10deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnRight_Step2[4]); //15deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnRight_Step2[5]); //-15deg;

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnRight_Step2[6]); //15deg;
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnRight_Step2[7]); //-15deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnRight_Step2[8]); //30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnRight_Step2[9]); //70deg;

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnRight_Step2[10]); //70deg;
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnRight_Step2[11]); //30deg
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 12;
                }
                break;
            case 12:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnRight_Step3[0]); //10deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnRight_Step3[1]); //-25deg               
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnRight_Step3[2]); //25deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnRight_Step3[3]); //-10deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnRight_Step3[4]); //0deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnRight_Step3[5]); //-15deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnRight_Step3[6]); //15deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnRight_Step3[7]); //0deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnRight_Step3[8]);//30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnRight_Step3[9]); //70deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnRight_Step3[10]); //70deg
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnRight_Step3[11]);//30deg
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 13;
                }
                break;
            case 13:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnRight_Step4[0]); //25deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnRight_Step4[1]); //-10deg                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnRight_Step4[2]); //10deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnRight_Step4[3]);//-25deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnRight_Step4[4]); //0deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnRight_Step4[5]); //-15deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnRight_Step4[6]); //15deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnRight_Step4[7]); //0deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnRight_Step4[8]); //70deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnRight_Step4[9]); //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnRight_Step4[10]); //30deg
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnRight_Step4[11]); //70deg
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 10;
                }
                break;
            case 20:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step6[0]);//588;// 10deg //557;
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step6[1]);//512;//0deg
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step6[2]);//512;//0deg
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 5;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step6[3]);//481; // -10deg //467;

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step6[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step6[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step6[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step6[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 5;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step6[8]);//604; //30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step6[9]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step6[10]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step6[11]);//604; //30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 21;
                }
                break;
            case 21:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step5[0]);//558; //10 deg //557;//
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step5[1]);//388; //-40 deg //392;
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step5[2]);//635; //40deg //632;
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step5[3]);//481; //-10deg //467;//

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step5[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step5[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step5[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step5[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step5[8]);//604;//30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step5[9]);//681; // 55deg //679;

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 1;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step5[10]);//681; // 55deg //679;//

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step5[11]);//604; //30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 22;
                }
                break;
            case 22:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step4[0]);//512; //0deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step4[1]);//388; // -40deg //392;//
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step4[2]);//635; //40deg //632;//
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 5;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step4[3]);//512; //0deg

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step4[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step4[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step4[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step4[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step4[8]);//604;// 30deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 1;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step4[9]);//635; //40deg //634;//

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 1;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step4[10]);//635; // 40deg //634;//

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step4[11]);//604;// 30deg
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 23;
                }
                break;
            case 23:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step3[0]);//512;// 0deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step3[1]);//435; //-25 deg //437;
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step3[2]);//588; //25 deg //587;
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step3[3]);//512;// 0deg

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step3[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step3[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step3[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step3[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 5;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step3[8]);//681; //55deg //679;

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step3[9]);//604; //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step3[10]);//604; //30deg

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 5;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step3[11]);//681; //55deg //679;
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 24;
                }
                break;
            case 24:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step2[0]);//650; // 45deg //647;//
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step2[1]);//435; //-25deg //437;//
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 1;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step2[2]);//588; // 25deg //587;//
                
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step2[3]);//373;// -45deg//387;//

                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step2[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step2[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step2[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step2[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step2[8]);//681; //55 deg //679;//

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step2[9]);//604; //30 deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step2[10]);//604; //30deg

                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step2[11]);//681; // 55deg //679;//
               
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 25;
                }

                break;
            case 25:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dFrontMove_Step1[0]);//602;//30deg
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dFrontMove_Step1[1]);//482; //-10deg                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dFrontMove_Step1[2]);//542; //10 deg
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dFrontMove_Step1[3]);//419;//432;// -30deg
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 5;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dFrontMove_Step1[4]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dFrontMove_Step1[5]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dFrontMove_Step1[6]);//512; //0 deg
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 5;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dFrontMove_Step1[7]);//512; //0 deg

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dFrontMove_Step1[8]);//635; //40deg

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dFrontMove_Step1[9]);//604; //30deg

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dFrontMove_Step1[10]);//604; //30deg
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dFrontMove_Step1[11]);//635;//40 deg
           
                SJOG_cmd.request.PlayTime = 20;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(100000);
                    cmd = 20;
                }
                break;
            case 30:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 5;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnLeft_Step1[0]);
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 1;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnLeft_Step1[1]);              
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnLeft_Step1[2]);
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnLeft_Step1[3]);
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnLeft_Step1[4]);

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnLeft_Step1[5]);

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnLeft_Step1[6]);
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnLeft_Step1[7]);

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnLeft_Step1[8]);

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnLeft_Step1[9]);

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnLeft_Step1[10]);
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnLeft_Step1[11]);
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 31;
                }
                break;
            case 31:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnLeft_Step2[0]);
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnLeft_Step2[1]);                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnLeft_Step2[2]);
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnLeft_Step2[3]);
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnLeft_Step2[4]);

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnLeft_Step2[5]);

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnLeft_Step2[6]);
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnLeft_Step2[7]);

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnLeft_Step2[8]);

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnLeft_Step2[9]);

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnLeft_Step2[10]);
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnLeft_Step2[11]);
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 32;
                }
                break;
            case 32:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnLeft_Step3[0]);
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnLeft_Step3[1]);                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnLeft_Step3[2]);
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnLeft_Step3[3]);
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnLeft_Step3[4]);

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnLeft_Step3[5]);

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnLeft_Step3[6]);
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnLeft_Step3[7]);

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnLeft_Step3[8]);

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnLeft_Step3[9]);

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnLeft_Step3[10]);
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnLeft_Step3[11]);
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);
                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 33;
                }
                break;
            case 33:
                SJOG_cmd.request.ID_Arr[0] = 1;
                SJOG_cmd.request.LED_Arr[0] = 1;
                SJOG_cmd.request.TargetPosition_Arr[0] = Set_AngleToCount(m_dTurnLeft_Step4[0]);
                
                SJOG_cmd.request.ID_Arr[1] = 2;
                SJOG_cmd.request.LED_Arr[1] = 5;
                SJOG_cmd.request.TargetPosition_Arr[1] = Set_AngleToCount(m_dTurnLeft_Step4[1]);                
                       
                SJOG_cmd.request.ID_Arr[2] = 3;
                SJOG_cmd.request.LED_Arr[2] = 5;
                SJOG_cmd.request.TargetPosition_Arr[2] = Set_AngleToCount(m_dTurnLeft_Step4[2]);
                    
                SJOG_cmd.request.ID_Arr[3] = 4;
                SJOG_cmd.request.LED_Arr[3] = 1;
                SJOG_cmd.request.TargetPosition_Arr[3] = Set_AngleToCount(m_dTurnLeft_Step4[3]);
            
                SJOG_cmd.request.ID_Arr[4] = 5;
                SJOG_cmd.request.LED_Arr[4] = 1;
                SJOG_cmd.request.TargetPosition_Arr[4] = Set_AngleToCount(m_dTurnLeft_Step4[4]);

                SJOG_cmd.request.ID_Arr[5] = 6;
                SJOG_cmd.request.LED_Arr[5] = 5;
                SJOG_cmd.request.TargetPosition_Arr[5] = Set_AngleToCount(m_dTurnLeft_Step4[5]);

                SJOG_cmd.request.ID_Arr[6] = 7;
                SJOG_cmd.request.LED_Arr[6] = 5;
                SJOG_cmd.request.TargetPosition_Arr[6] = Set_AngleToCount(m_dTurnLeft_Step4[6]);
            
                SJOG_cmd.request.ID_Arr[7] = 8;
                SJOG_cmd.request.LED_Arr[7] = 1;
                SJOG_cmd.request.TargetPosition_Arr[7] = Set_AngleToCount(m_dTurnLeft_Step4[7]);

                SJOG_cmd.request.ID_Arr[8] = 9;
                SJOG_cmd.request.LED_Arr[8] = 1;
                SJOG_cmd.request.TargetPosition_Arr[8] = Set_AngleToCount(m_dTurnLeft_Step4[8]);

                SJOG_cmd.request.ID_Arr[9] = 10;
                SJOG_cmd.request.LED_Arr[9] = 5;
                SJOG_cmd.request.TargetPosition_Arr[9] = Set_AngleToCount(m_dTurnLeft_Step4[9]);

                SJOG_cmd.request.ID_Arr[10] = 11;
                SJOG_cmd.request.LED_Arr[10] = 5;
                SJOG_cmd.request.TargetPosition_Arr[10] = Set_AngleToCount(m_dTurnLeft_Step4[10]);
            
                SJOG_cmd.request.ID_Arr[11] = 12;
                SJOG_cmd.request.LED_Arr[11] = 1;
                SJOG_cmd.request.TargetPosition_Arr[11] = Set_AngleToCount(m_dTurnLeft_Step4[11]);
           
               
                SJOG_cmd.request.PlayTime = 30;
                SJOG_cmd.request.Total_Axis = 12;

                SJOGCmd_client.call(SJOG_cmd);

                if(m_Stop)
                    cmd = 9;
                else
                {
                    usleep(200000);
                    cmd = 30;
                }
                break;
            default:
                break;
        }
 
        last_time = current_time;
        //ROS_INFO("Time: %.2f\n",dt);
        loop_rate.sleep();
    }

    return 0;
}

