#include <ros/ros.h>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


bool simul = false;
bool new_post = false;

int goalPos[2] = {0, 0};
int goalSpeeds = (0.078*((4095/360)*(180/M_PI)));
int motors = 2;
int minLimits[2] = {0, 0};
int maxLimits[2] = {4095, 4095};
float giro[2] = {0,0};

float goalPos_simul[2] = {0.0, 0.0};
float goalSpeeds_simul[2] = {0.3, 0.3};

int zero_head[2] = {2048, 2048};
//float offset = -0.07671; // This for help me carry
float offset = -0.04; // This is for p and g 
float offsetReadSimul = -0.04;

float save[2] = {0,0};

uint16_t info[2] = {3071, 0};

void callbackHeadGoalPose(const std_msgs::Float32MultiArray::ConstPtr &msg){
    // std::cout << "head_node.-> Reciving new goal head pose." << std::endl;
    if(!(msg->data.size() == 2))
        std::cout << "Can not process the goal poses for the head" << std::endl;
    else
    { 
        goalPos_simul[0] = msg->data[0];
        goalSpeeds_simul[0] = 0.1;
        goalPos_simul[1] = msg->data[1];
        goalSpeeds_simul[1] = 0.1;

        giro[0] = goalPos_simul[0] - save[0];
        giro[1] = goalPos_simul[1] - save[1];
        save[0] = goalPos_simul[0];
        save[1] = goalPos_simul[1];

        goalPos[0] = int(giro[0]*((4096/360)*(180/M_PI)) + int(info[0]));
        goalPos[1] = int(giro[1]*((4096/360)*(180/M_PI)) + int(info[1]));        

        //std::cout<<"Giro 1: "<< int(giro[0]*((4096/360)*(180/M_PI))) <<std::endl;
        //std::cout<<"Giro 2: "<< int(giro[1]*((4096/360)*(180/M_PI))) <<std::endl;

        std::cout<<"Valor Final 1: "<< goalPos[0] <<std::endl;
        std::cout<<"Valor Final 2: "<< goalPos[1] <<std::endl;  
        if(goalPos[0] >= minLimits[0] && goalPos[0] <= maxLimits[0] && goalPos[1] >= minLimits[1] && goalPos[1] <= maxLimits[1])
            new_post =true;
        
    }
}

void callback_simulated(const std_msgs::Bool::ConstPtr &msg){
    simul = msg->data;
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "head_sdk");
    ros::NodeHandle n;
    ros::NodeHandle node("~"); 
    std::string port;
    int addr = 30;
    int baudRate;
    new_post = false;
    simul = false;   
    
    std::cout << "INITIALIZING SERGIO" << std::endl;
    node.param("baudrate", baudRate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);    
    portHandler->setBaudRate(baudRate);


    uint8_t  dxl_error_write = 0;
    uint8_t  dxl_error_read      = 0;
    int      dxl_comm_result_write = COMM_TX_FAIL;
    int      dxl_comm_result_read = COMM_TX_FAIL;

    ros::Subscriber subGoalPos = n.subscribe("/hardware/head/goal_pose", 1, callbackHeadGoalPose);
    ros::Subscriber subSimul = n.subscribe("/simulated", 1, callback_simulated); 
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher pubHeadPose = n.advertise<std_msgs::Float32MultiArray>("head/current_pose", 1);
    //ros::Publisher pubBattery = n.advertise<std_msgs::Float32>("/hardware/robot_state/head_battery", 1);

    ros::Rate rate(30);

    std::vector<int> ids;
    ids.push_back(2);
    ids.push_back(3);
    
    //iniciando los en la posicion 0
    for(int i = 0; i < motors; i++)
    {
        dxl_comm_result_write = packetHandler->write2ByteTxRx(portHandler, ids[i], addr,zero_head[i], &dxl_error_write);
    }

    //float bitsPerRadian = 4095.0/360.0*180.0/M_PI;

    std::string names[2] = {"pan_connect","tilt_connect"};
    float positions[2] = {0,0};

    std_msgs::Float32MultiArray msgCurrPose;
    msgCurrPose.data.resize(2);
    
    sensor_msgs::JointState jointStates;
    jointStates.name.insert(jointStates.name.begin(), names, names + 2);
    jointStates.position.insert(jointStates.position.begin(), positions, positions + 2);

    float Pos_simul[2] = {0.0, 0.0};
    float deltaPos_simul[2] = {0.0, 0.0};
    float deltaPos[2] = {0.0, 0.0};
    float Pos[2] = {0.0, 0.0};
    for(int i = 0; i < 2; i++)
    {
        goalPos_simul[i] = 0.0;
        goalSpeeds_simul[i] = 0.1;
    }


    while(ros::ok())
    {    
        for(int j = 0; j < motors; j++)
        {
            dxl_comm_result_read = packetHandler->read2ByteTxRx(portHandler, ids[j],36,&info[j], &dxl_error_read);
            Pos[j] = int(info[j]);
        }        
        if(new_post)
        {
            for(int i = 0; i < motors; i++)
            {
                deltaPos[i] = goalPos[i] - Pos[i];///incrementar Pos hasta llegar al valor
                
                if(deltaPos[i] > goalSpeeds)
                    deltaPos[i] = goalSpeeds;
                if(deltaPos[i] < -goalSpeeds)
                    deltaPos[i] = -goalSpeeds;
                Pos[i] += deltaPos[i];
                //std::cout<<"Pos: "<<deltaPos[i]<<std::endl; 
                dxl_comm_result_write = packetHandler->write2ByteTxRx(portHandler, ids[i], addr,Pos[i], &dxl_error_write);
            }
            if(deltaPos < 0)
                new_post = false;


            /*for(int i = 0; i < motors; i++)
            {
                deltaPos[i] = goalPos[i];
                std::cout<<"deltaPos: "<< deltaPos[i] <<std::endl; 
                std::cout<<"Speeds: "<< goalSpeeds <<std::endl;
                while( deltaPos[i] != 0 )
                {                      
                    if (goalPos[i] > 0)
                    {
                        Pos[i] += goalSpeeds;
                        deltaPos[i] -= goalSpeeds;
                    }
                    else
                    {
                        Pos[i] -= goalSpeeds;
                        deltaPos[i] += goalSpeeds;
                    }
                    std::cout<<"Pos: "<<Pos[i]<<std::endl; 
                    dxl_comm_result_write = packetHandler->write2ByteTxRx(portHandler, ids[i], addr,Pos[i], &dxl_error_write);
                    //deltaPos[i] -= goalSpeeds;
                }


                //deltaPos[i] = goalPos[i] - Pos[i];///incrementar Pos hasta llegar al valor
            }
            new_post = false;//*/
        }
            

        for(int i = 0; i < 2; i++)
        {
            deltaPos_simul[i] = goalPos_simul[i] - Pos_simul[i];///incrementar Pos hasta llegar al valor
            if(deltaPos_simul[i] > goalSpeeds_simul[i])
                deltaPos_simul[i] = goalSpeeds_simul[i];
            if(deltaPos_simul[i] < -goalSpeeds_simul[i])
                deltaPos_simul[i] = -goalSpeeds_simul[i];
            Pos_simul[i] += deltaPos_simul[i];
            if(i == 0)
                jointStates.position[i] = Pos_simul[i];
            else
            { 
                jointStates.position[i] = -Pos_simul[i];
            }
            msgCurrPose.data[i] = Pos_simul[i]; 
        }
               
        jointStates.header.stamp = ros::Time::now();
           
        pubHeadPose.publish(msgCurrPose);
        joint_pub.publish(jointStates);
       
            

        rate.sleep();
        ros::spinOnce();
    }

    portHandler->closePort();
    return 1;
}