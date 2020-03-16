/*
HerkuleX_12DOF_Robot_node
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
using namespace std;

//HerkuleX...
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

// MoveIt!
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

/* Moveit .......*/
//robot_model_loader::RobotModelLoader robot_model_loader("HerkuleX");
//robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
//ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


/*Coomand srv _ Client************************/
HerkuleX::HerkuleX_RegisterCommand Register_cmd[7];
HerkuleX::HerkuleX_PositionMove Position_cmd[7];
HerkuleX::HerkuleX_VelocityMove Velocity_cmd[7];
HerkuleX::HerkuleX_SJOG_Move SJOG_cmd;
HerkuleX::HerkuleX_IJOG_Move IJOG_cmd;

ros::ServiceClient Register_client;
ros::ServiceClient PositionCmd_client;
ros::ServiceClient VelocityCmd_client;
ros::ServiceClient SJOGCmd_client;
ros::ServiceClient IJOGCmd_client;
/********************************************/

/* Joint state publisher************************/
ros::Publisher joint_pub;
//Define the joint state
sensor_msgs::JointState joint_state;
float m_Joint_rad[7] = {0.0, };
/*HerkuleX_6DOF_Joint Subscriber************************/
ros::Subscriber Arm_sub;

/*HerkuleX RAM msg Subscriber************************/
ros::Subscriber RAM_Array_sub[7];

int m_iMode = 0;
bool m_bFlag_Jointcallback = false;

float m_Joint_Pose_rad[7] = {0.0, };
int m_Joint_TargetPose[7] = {0, };
int m_Joint_TargetTime[7] = {60, 60, 60, 60, 60, 60, 60}; //Play Time(60*11.2ms): 672ms

//Step Table
double m_dHome[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double dJoint_cal_POS[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

pthread_t p_thread;
int cmd = 0;
int Value = 0;
bool m_Stop = false;

void *t_function(void *data)
{
    while(1)
    {
        if(!m_bFlag_Jointcallback)
        {
            for(int i=0; i<7; i++) //Axis Total: 6(Arm) + 1(Gripper) = 7
            {
                Register_cmd[i].request.command = "RAM_RegisterData_Read"; //"RAM_RegisterData_Read_All";
                Register_cmd[i].request.Model_Num = 0; //DRS-0201
                Register_cmd[i].request.ID = i+1;
                Register_cmd[i].request.Addr = RAM_CALIBRATED_POSITION; //RAM_ID; 
                Register_cmd[i].request.Value = 1;//RAM_LAST;
                Register_client.call(Register_cmd[i]);
                usleep(10000);
            }
        }
        else
        {
            usleep(10000);
        }

    }

    pthread_cancel(p_thread); //Thread kill
}

int Set_DegreeToCount(double m_dDegree)
{
    int iResult = 0;
    iResult = (int)(m_dDegree/0.325) +512; //DRS-0101의 경우, 소수점은 버림
    return iResult;
}

int Set_RadianToCount(float m_fRadian)
{
    int iResult = 0;
    float m_fdegree = (m_fRadian * 180.0)/M_PI;
    iResult = (int)(m_fdegree/0.325) +512; //DRS-0101의 경우, 소수점은 버림
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

void subJoint1_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[0] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[0] = m_Joint_rad[0];
    joint_pub.publish(joint_state);

}

void subJoint2_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[1] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[1] = m_Joint_rad[1];
    joint_pub.publish(joint_state);

}

void subJoint3_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[2] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[2] = m_Joint_rad[2];
    joint_pub.publish(joint_state);

}

void subJoint4_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[3] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[3] = m_Joint_rad[3];
    joint_pub.publish(joint_state);

}

void subJoint5_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[4] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[4] = m_Joint_rad[4];
    joint_pub.publish(joint_state);

}

void subJoint6_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[5] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[5] = m_Joint_rad[5];
    joint_pub.publish(joint_state);

}

void subJoint7_Info(const HerkuleX::HerkuleX_Info_RAM::ConstPtr& msg)
{
    //ROS_INFO("RAM_Info_msg %d\n", msg->pHerkuleX_RAM.size());
    for (int i = 0; i < msg->pHerkuleX_RAM.size(); i++)
    {
        const HerkuleX::MsgHerkuleX_RAM &RAM_msg = msg->pHerkuleX_RAM[i];
        m_Joint_rad[6] = Set_CountToRadian(RAM_msg.CalibratedPosition);
        //ROS_INFO("Pan Position(rad): %.2f", m_fPan_rad);
    }
    
    joint_state.position[6] = m_Joint_rad[6];
    joint_pub.publish(joint_state);

}

void Arm_Joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Position -> Rad -> Target Pose
    for(int i=0; i<7; i++)
    {
        m_Joint_Pose_rad[i] = msg->position[i];
        m_Joint_TargetPose[i] = Set_RadianToCount(m_Joint_Pose_rad[i]);
        //ROS_INFO("m_Joint_TargetPose[%d]: %d", i, m_Joint_TargetPose[i]);

    }

    if(m_bFlag_Jointcallback)
    {
        for(int j=0; j<7; j++)
        {
            Position_cmd[j].request.ID = j+1;
            Position_cmd[j].request.LED = 1; //green
            Position_cmd[j].request.PlayTime = m_Joint_TargetTime[j];
            Position_cmd[j].request.TargetPosition = m_Joint_TargetPose[j];
            Position_cmd[j].request.JogMode = 0;

            PositionCmd_client.call(Position_cmd[j]);
        }
        
    }

}


//int main (int argc, char** argv)
int main (int argc, char* argv[])
{
    ros::init(argc, argv, "HerkuleX_6DOF_Arm_node");

    //Sellect Check Mode//
	m_iMode = atoi(argv[1]);
	ROS_INFO("Mode: %d", m_iMode);

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

    //HerkuleX RAM msg Subscriber/////////////////////////////////////////////////////
    ros::NodeHandle R1sub_h;
    RAM_Array_sub[0] = R1sub_h.subscribe("Info_RAM_ID_1", 10, subJoint1_Info);
    RAM_Array_sub[1] = R1sub_h.subscribe("Info_RAM_ID_2", 10, subJoint2_Info);
    RAM_Array_sub[2] = R1sub_h.subscribe("Info_RAM_ID_3", 10, subJoint3_Info);
    RAM_Array_sub[3] = R1sub_h.subscribe("Info_RAM_ID_4", 10, subJoint4_Info);
    RAM_Array_sub[4] = R1sub_h.subscribe("Info_RAM_ID_5", 10, subJoint5_Info);
    RAM_Array_sub[5] = R1sub_h.subscribe("Info_RAM_ID_6", 10, subJoint6_Info);
    RAM_Array_sub[6] = R1sub_h.subscribe("Info_RAM_ID_7", 10, subJoint7_Info);

    //Decleare a joint state publisher
    ros::NodeHandle Jpub_h;
    joint_pub = Jpub_h.advertise<sensor_msgs::JointState>("joint_states",1);
    
    joint_state.name.push_back("joint1");
    joint_state.name.push_back("joint2");
    joint_state.name.push_back("joint3");
    joint_state.name.push_back("joint4");
    joint_state.name.push_back("joint5");
    joint_state.name.push_back("joint6");
    joint_state.name.push_back("joint7");

    unsigned int n = joint_state.name.size();
    joint_state.position.resize(n);
    joint_state.velocity.resize(n);
    joint_state.effort.resize(n);

    //HerkuleX 6DOF Joint Subscriber//////////////////////////////////////
    ros::NodeHandle Arm_h;
    Arm_sub = Arm_h.subscribe("joint_states", 10, Arm_Joint_callback);


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

    for(int i=0; i<7; i++)
    {
        Register_cmd[i].request.command = "SERVO_ON";
        Register_cmd[i].request.ID = i+1;
        Register_client.call(Register_cmd[i]);
        usleep(10000); //10ms

    }

    for(int j=0; j<7; j++)
    {
        Position_cmd[j].request.ID = j+1;
        Position_cmd[j].request.LED = 5; //Yellow
        Position_cmd[j].request.PlayTime = 60; //Play Time(60*11.2ms): 672ms

        switch(j)
        {
            case 0:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 1:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 2:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 3:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 4:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 5:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 6:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            case 7:
                Position_cmd[j].request.TargetPosition = Set_DegreeToCount(m_dHome[j]);
                break;
            
        }

        Position_cmd[j].request.JogMode = 0;

        PositionCmd_client.call(Position_cmd[j]);

    }

    sleep(1);

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

                for(int i=0; i<7; i++)
                {
                    Position_cmd[i].request.LED = 7;
                    Position_cmd[i].request.TargetPosition = dJoint_cal_POS[i];
                    PositionCmd_client.call(Position_cmd[i]);
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

