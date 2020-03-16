/*
HerkuleX_feedback_Test_node
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h> //Joint msg
#include <math.h>
#include <unistd.h>

#include "HerkuleX_node.h"
#include "HerkuleX/MsgHerkuleX_RAM.h" //MSG
#include "HerkuleX/MsgHerkuleX_EEP.h" //MSG
#include "HerkuleX/HerkuleX_Info_RAM.h" //MSG_Array
#include "HerkuleX/HerkuleX_Info_EEP.h" //MSG_Array
#include "HerkuleX/HerkuleX_RegisterCommand.h" //SRV
#include "HerkuleX/HerkuleX_PositionMove.h" //SRV
#include "HerkuleX/HerkuleX_VelocityMove.h" //SRV
using namespace std;
#include <pthread.h>

ros::Time current_time, last_time;
/*Coomand srv _ Client************************/
HerkuleX::HerkuleX_RegisterCommand Register_cmd;
HerkuleX::HerkuleX_PositionMove Position_cmd[2];
HerkuleX::HerkuleX_VelocityMove Velocity_cmd[2];
ros::ServiceClient Register_client;
ros::ServiceClient PositionCmd_client;
ros::ServiceClient VelocityCmd_client;
/********************************************/

/*Pan&Tilt Joint state publisher************************/
ros::Publisher joint_pub;
//Define the joint state
sensor_msgs::JointState joint_state;
float m_fPan_rad = 0.0;
float m_fTilt_rad = 0.0;
/*HerkuleX Pan&Tilt Joint Subscriber************************/
ros::Subscriber PanTilt_sub;

/*HerkuleX RAM msg Subscriber************************/
ros::Subscriber RAM_Array_sub[2];
/*OpenCV Subscriber************************/
ros::Subscriber CV3_Result_X;
ros::Subscriber CV3_Result_Y;

int iPixel_x = 0;
int iPixel_y = 0;
int iTargetPos_Pan = 0;
int iTargetPos_Tilt = 0;
double dPan_Gain = 0.5;
double dTilt_Gain = 0.5;
double dPan_cal_POS = 0.0;
double dTilt_cal_POS = 0.0;

int iCenterImage_W = 320;
int iCenterImage_H = 240;
/*****************************/
int m_iMode = 0;
bool m_bFlag_Jointcallback = false;

pthread_t p_thread;
void *t_function(void *data)
{

    while(1)
    {
        for(int i=0; i<2; i++) //Axis Total: 2 (Pan: ID 1, Tilt: ID 2)
        {
            Register_cmd.request.command = "RAM_RegisterData_Read"; //"RAM_RegisterData_Read_All";
            Register_cmd.request.Model_Num = 0; //DRS-0101
            Register_cmd.request.ID = i+1;
            Register_cmd.request.Addr = RAM_CALIBRATED_POSITION; //RAM_ID; 
            Register_cmd.request.Value = 1;//RAM_LAST;
            Register_client.call(Register_cmd);
            usleep(10000);
        }
    }

    pthread_cancel(p_thread); //Thread kill
}


int Set_DegreeToCount(double m_dDegree)
{
    int iResult = 0;
    iResult = (int)(m_dDegree/0.325) + 512; //DRS-0101의 경우, 소수점은 버림
    return iResult;
}

int Set_RadianToCount(float m_fRadian)
{
    int iResult = 0;
    float m_fdegree = (m_fRadian * 180.0)/M_PI;
    iResult = (int)(m_fdegree/0.325) + 512; //DRS-0101의 경우, 소수점은 버림
    return iResult;
}

float Set_CountToDegree(unsigned int ucount)
{
    float dResult = 0.0;
    //Count -> Degree
    dResult = (ucount - 512.0) * 0.325; //DRS-0101의 경우
    return dResult;
}

float Set_CountToRadian(unsigned int ucount)
{
    float dResult = 0.0;
    //Count -> Degree
    dResult = (ucount - 512.0) * 0.325; //DRS-0101의 경우
    //Degree -> Radian
    dResult = (dResult * M_PI)/180.0;
    return dResult;
}

void CV3_Result_X_callback(const std_msgs::String::ConstPtr& msg)
{
    
    //ROS_INFO("X=%s", msg->data.c_str());
    iPixel_x = std::atoi(msg->data.c_str());
    //ROS_INFO("X = %d", iPixel_x);

}
void CV3_Result_Y_callback(const std_msgs::String::ConstPtr& msg)
{
    
    //ROS_INFO("Y=%s", msg->data.c_str());
    iPixel_y = std::atoi(msg->data.c_str());
    //ROS_INFO("Y = %d", iPixel_y);

}

void subPan_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_fPan_rad = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    if(!m_bFlag_Jointcallback)
    {
        joint_state.position[0] = m_fPan_rad;
        joint_pub.publish(joint_state);
    }

}

void subTilt_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_fTilt_rad = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Tilt Position(rad): %.2f", m_fTilt_rad);
    }

    if(!m_bFlag_Jointcallback)
    {
        joint_state.position[1] = m_fTilt_rad;
        joint_pub.publish(joint_state);
    }

}

void PanTilt_Joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    float m_fPan_Pose_rad = msg->position[0];
    float m_fTilt_Pose_rad = msg->position[1];
    //ROS_INFO("Pan_POSE: %.2f", m_fPan_Pose_rad);

    //Radian --> Count
    int m_iPan_TargetPose = Set_RadianToCount(m_fPan_Pose_rad);
    int m_iTilt_TargetPose = Set_RadianToCount(m_fTilt_Pose_rad);

    
    if(m_bFlag_Jointcallback)
    {
        Position_cmd[0].request.ID = 1;
        Position_cmd[0].request.LED = 1; //green
        Position_cmd[0].request.PlayTime = 10; //Play Time(10*11.2ms): 112ms
        Position_cmd[0].request.TargetPosition = m_iPan_TargetPose;
        Position_cmd[0].request.JogMode = 0;

        Position_cmd[1].request.ID = 2;
        Position_cmd[1].request.LED = 1; //green
        Position_cmd[1].request.PlayTime = 10; //Play Time(10*11.2ms): 112ms.
        Position_cmd[1].request.TargetPosition = m_iTilt_TargetPose;
        Position_cmd[1].request.JogMode = 0;    

        PositionCmd_client.call(Position_cmd[0]);
        PositionCmd_client.call(Position_cmd[1]);
    }
    

}


//int main (int argc, char** argv)
int main (int argc, char* argv[])
{
    ros::init(argc, argv, "HerkuleX_PanTilt_node");

    //Sellect Check Mode//
	m_iMode = atoi(argv[1]);
	ROS_INFO("Mode: %d", m_iMode);

	ros::NodeHandle Rcmd_h;
	ros::NodeHandle Pcmd_h;
	ros::NodeHandle Vcmd_h;

    //HerkuleX RAM msg Subscriber/////////////////////////////////////////////////////
    ros::NodeHandle R1sub_h;
    ros::NodeHandle R2sub_h;
    RAM_Array_sub[0] = R1sub_h.subscribe("Info_RAM_ID_1", 100, subPan_Info);
    RAM_Array_sub[1] = R2sub_h.subscribe("Info_RAM_ID_2", 100, subTilt_Info);

    //Decleare a joint state publisher
    ros::NodeHandle Jpub_h;
    joint_pub = Jpub_h.advertise<sensor_msgs::JointState>("joint_states",1);

    joint_state.name.push_back("joint1"); //Pan_Joint
    joint_state.name.push_back("joint2"); //Tilt_Joint
    unsigned int n = joint_state.name.size();
    joint_state.position.resize(n);
    joint_state.velocity.resize(n);
    joint_state.effort.resize(n);

    //HerkuleX PanTilt Joint Subscriber//////////////////////////////////////
    ros::NodeHandle PT_h;
    PanTilt_sub = PT_h.subscribe("joint_states", 10, PanTilt_Joint_callback);

    //OpenCV Subscriber/////////////////////////////////////////////////////
    ros::NodeHandle sh1;
    ros::NodeHandle sh2;
    CV3_Result_X = sh1.subscribe("CV3_Result_X", 10, CV3_Result_X_callback);
    CV3_Result_Y = sh2.subscribe("CV3_Result_Y", 10, CV3_Result_Y_callback);
	
    //Command ServiceClient  
    Register_client	   = Rcmd_h.serviceClient<HerkuleX::HerkuleX_RegisterCommand>("Register_cmd");
	PositionCmd_client = Pcmd_h.serviceClient<HerkuleX::HerkuleX_PositionMove>("Position_cmd");
	VelocityCmd_client = Vcmd_h.serviceClient<HerkuleX::HerkuleX_VelocityMove>("Velocity_cmd");

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
    Register_cmd.request.command = "SERVO_ON";
    Register_cmd.request.ID = 1;
    Register_client.call(Register_cmd);
    usleep(10000);
    Register_cmd.request.command = "SERVO_ON";
    Register_cmd.request.ID = 2;
    Register_client.call(Register_cmd);
    usleep(10000);

    Position_cmd[0].request.ID = 1;
    Position_cmd[0].request.LED = 2; //Blue
    Position_cmd[0].request.PlayTime = 60; //Play Time(*11.2ms): 60.
    Position_cmd[0].request.TargetPosition = 512;
    Position_cmd[0].request.JogMode = 0;
    
    Position_cmd[1].request.ID = 2;
    Position_cmd[1].request.LED = 2; //Blue
    Position_cmd[1].request.PlayTime = 60; //Play Time(*11.2ms): 60.
    Position_cmd[1].request.TargetPosition = 512;
    Position_cmd[1].request.JogMode = 0;
    
    PositionCmd_client.call(Position_cmd[0]);
    PositionCmd_client.call(Position_cmd[1]);
    usleep(10000);


    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(100.0); //HZ


    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        joint_state.header.stamp = current_time;

        double dt = (current_time - last_time).toSec();

        switch(m_iMode)
        {
            case 0:
                break;
            case 1:
                //ROS_INFO("Manual Mode !");
                m_bFlag_Jointcallback = true;
                break;
            case 2:
                //ROS_INFO("Auto Mode !");
                m_bFlag_Jointcallback = false;
                //calc//
                iTargetPos_Pan = iPixel_x - iCenterImage_W;
                dPan_cal_POS = iTargetPos_Pan * dPan_Gain;
                //printf("dPan_cal_POS = %d \n", (int)dPan_cal_POS);

                iTargetPos_Tilt = iPixel_y - iCenterImage_H;
                dTilt_cal_POS = iTargetPos_Tilt * dTilt_Gain;
                //printf("dTilt_cal_POS = %d \n", (int)dTilt_cal_POS);

                Position_cmd[0].request.LED = 7;
                Position_cmd[0].request.TargetPosition = 512 + (dPan_cal_POS);
                Position_cmd[1].request.LED = 7;
                Position_cmd[1].request.TargetPosition = 512 + (dTilt_cal_POS);

                PositionCmd_client.call(Position_cmd[0]);
                PositionCmd_client.call(Position_cmd[1]);

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

