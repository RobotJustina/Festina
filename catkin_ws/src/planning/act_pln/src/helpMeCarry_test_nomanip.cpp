#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
//#include "string"
#include <sensor_msgs/LaserScan.h>

#define SM_INIT 0
#define SM_INSTRUCTIONS 5
#define SM_WAIT_FOR_OPERATOR 10
#define SM_MEMORIZING_OPERATOR 20
#define SM_WAIT_FOR_LEGS_FOUND 25
#define SM_FOLLOWING_PHASE 30
#define SM_BRING_GROCERIES 40
#define SM_BRING_GROCERIES_CONF 41
#define SM_BRING_GROCERIES_TAKE 42
#define SM_BAG_DELIVERY 50
#define SM_BAG_DELIVERY_PLACE 60
#define SM_LOOKING_HELP 70
#define SM_GUIDING_ASK 75
#define SM_GUIDING_HELP 80
#define SM_GUIDING_MEMORIZING_OPERATOR 90
#define SM_GUIDING_MEMORIZING_OPERATOR_ELF 91
#define SM_GUIDING_MEMORIZING_OPERATOR_SAY 92
#define SM_GUIDING_PHASE 100
#define SM_GUIDING_STOP 101
#define SM_GUIDING_CAR 102
#define SM_OPEN_DOOR 103
#define SM_FINAL_STATE 110
#define SM_HOKUYO_TEST 1000


#define MAX_ATTEMPTS_RECOG 3
#define MAX_ATTEMPTS_CONF 3

sensor_msgs::LaserScan laser;
std::vector<float> laser_ranges;
bool door_isopen=false;
bool door_loc=false;
int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
float laser_l=0;


vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, bool &recognized)
{
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    vision_msgs::VisionFaceObjects lastRecognizedFaces;

    do
    {
        lastRecognizedFaces = JustinaVision::getFaces();
        
        if(previousSize == 1)
            sameValue ++;
        
        if (sameValue == 3)
            recognized = true;

        else
        {
            previousSize = lastRecognizedFaces.recog_faces.size();
            recognized = false;
        }

        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && !recognized);

    std::cout << "recognized:" << recognized << std::endl;
    return lastRecognizedFaces;
}

void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    range=msg->ranges.size();
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    std::cout<<"Range Size: "<< range << "\n ";
    std::cout<<"Range Central: "<< range_c << "\n ";
    std::cout<<"Range Initial: "<< range_i << "\n ";
    std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(msg->ranges[i] > 0 && msg->ranges[i] < 4){ 
            laser_l=laser_l+msg->ranges[i];    
            cont_laser++;
        }
    }
    std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 2.0){
        door_isopen=true;
    }
    else{
        door_isopen=false;
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    std::cout << system("pacmd set-default-source alsa_input.pci-0000_00_1f.3.analog-stereo") << std::endl;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;

    //int c_point=0,i=1;
    bool is_location;
    int nextState = SM_INIT;
    bool fail = false;
    bool success = false;
    float x, y ,z;
    std::stringstream ss;
    std::vector<std::string> tokens;
    int attemptsRecogLoc = 0;
    int attemptsConfLoc = 0;

    std::string lastRecoSpeech;
    std::string location="center table";
    std::vector<std::string> validCommandsStop;
    std::vector<std::string> validCommandsTake;
    validCommandsStop.push_back("here is the car");
    validCommandsStop.push_back("stop follow me");
    
    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;

    //places
    validCommandsTake.push_back("take this bag to the bedroom");
    validCommandsTake.push_back("get this bag to the bedroom");
    location="bedroom";
    
    validCommandsTake.push_back("take this bag to the bed");
    validCommandsTake.push_back("get this bag to the bed");
    location="bed";

    validCommandsTake.push_back("take this bag to the desk");
    validCommandsTake.push_back("get this bag to the desk");
    location="desk";

    validCommandsTake.push_back("take this bag to the side table");
    validCommandsTake.push_back("get this bag to the side table");
    location="side_table";
    
    validCommandsTake.push_back("take this bag to the living room");
    validCommandsTake.push_back("get this bag to the living room");
    location="living_room";

    validCommandsTake.push_back("take this bag to the couch");
    validCommandsTake.push_back("get this bag to the couch");
    location="couch";

    validCommandsTake.push_back("take this bag to the end table");
    validCommandsTake.push_back("get this bag to the end table");
    location="end_table";
    
    validCommandsTake.push_back("take this bag to the bookcase");
    validCommandsTake.push_back("get this bag to the bookcase");
    location="bookcase";

    validCommandsTake.push_back("take this bag to the dining room");
    validCommandsTake.push_back("get this bag to the dining room");
    location="dining_room";

    validCommandsTake.push_back("take this bag to the dining table");
    validCommandsTake.push_back("get this bag to the dining table");
    location="dining_table";

    validCommandsTake.push_back("take this bag to the kitchen");
    validCommandsTake.push_back("get this bag to the kitchen");
    location="kitchen";

    validCommandsTake.push_back("take this bag to the sink");
    validCommandsTake.push_back("get this bag to the sink");
    location="sink";

    validCommandsTake.push_back("take this bag to the dishwasher");
    validCommandsTake.push_back("get this bag to the dishwasher");
    location="dishwasher";

    validCommandsTake.push_back("take this bag to the counter");
    validCommandsTake.push_back("get this bag to the counter");
    location="counter";

    validCommandsTake.push_back("take this bag to the storage table");
    validCommandsTake.push_back("get this bag to the storage table");
    location="storage_table";

    validCommandsTake.push_back("take this bag to the cupboard");
    validCommandsTake.push_back("get this bag to the cupboard");
    location="cupboard";

    validCommandsTake.push_back("take this bag to the entrance");
    validCommandsTake.push_back("get this bag to the entrance");
    location="entrance";
    
    validCommandsTake.push_back("take this bag to the corridor");
    validCommandsTake.push_back("get this bag to the corridor");
    location="corridor";

    ros::Subscriber laser_subscriber;
    //laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, Callback_laser);  

    bool hokuyoRear = false;
    bool userConfirmation = false;
    bool follow_start=false;
    bool alig_to_place=true;
    int cont_z=3;

    vision_msgs::VisionFaceObjects faces;
    bool recog =false;
    int contChances=0;
    bool withLeftArm = false;

    //JustinaHRI::setInputDevice(JustinaHRI::RODE);
    //JustinaHRI::setOutputDevice(JustinaHRI::USB);
    //JustinaHRI::setVolumenInputDevice(JustinaHRI::RODE, 65000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::USB, 80000);
    JustinaTools::pdfStart("HelpMeCarry_Plans");

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {  

            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaHRI::waitAfterSay("I am ready for the help me carry test", 2000, minDelayAfterSay);
                JustinaHRI::loadGrammarSpeechRecognized("HelpMeCarry.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the Help me Carry Test");
                nextState = SM_INSTRUCTIONS;
                break;

            case SM_INSTRUCTIONS:
                std::cout << "State machine: SM_INSTRUCTIONS" << std::endl;
                JustinaHRI::waitAfterSay("Please tell me  come with me  and take me to the car, when we reach tell me here is the car. ", 1000);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                nextState=SM_WAIT_FOR_OPERATOR;
                break;    

            case SM_WAIT_FOR_OPERATOR:
                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("come with me" , 15000)){
                    nextState = SM_MEMORIZING_OPERATOR;
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Follow me command was recognized");
                }
                else                    
                    cont_z++;    		

                if(cont_z>3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Sorry I did not understand you, please repeat that", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                if(!follow_start){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay(" Please stand in front of me.", 3000, minDelayAfterSay);
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the search of human");
                    JustinaHRI::enableLegFinder(true);
                }
                else
                    JustinaHRI::enableLegFinder(true);    

                nextState=SM_WAIT_FOR_LEGS_FOUND;
                break;

            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    if(follow_start){
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("There you are, I am ready to follow you,please take me to the car", 4000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human was found with Hokuyo Laser");
                        JustinaHRI::startFollowHuman();
                        ros::spinOnce();
                        loop.sleep();
                        JustinaHRI::startFollowHuman();
                        nextState = SM_FOLLOWING_PHASE;
                    }
                    else{
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("There you are, take me to the car", 1000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                        JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human was found with Hokuyo Laser");
                        JustinaHRI::startFollowHuman();
                        follow_start=true;
                        nextState = SM_FOLLOWING_PHASE;
                    }
                }
                break;

            case SM_FOLLOWING_PHASE:
                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("here is the car") != std::string::npos || lastRecoSpeech.find("stop follow me") != std::string::npos){
                        JustinaTools::pdfAppend("HelpMeCarry_Plans", "Here is the car command was recognized");
                        JustinaTools::pdfAppend("HelpMeCarry_Plans", "Waiting for user confirmation");
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("is it the car location, please tell me robot yes, or robot no", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaHRI::waitForUserConfirmation(userConfirmation, 5000);
                        if(userConfirmation){
                            JustinaHRI::stopFollowHuman();
                            JustinaHRI::enableLegFinder(false);
                            JustinaKnowledge::addUpdateKnownLoc("car_location");	
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("I stopped", 2000, minDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                            JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot Yes command was recognized");
                            JustinaTools::pdfAppend("HelpMeCarry_Plans", "Saving the car location");
                            nextState = SM_BRING_GROCERIES;
                            cont_z=8;
                            break;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("Ok, please walk", 3000, maxDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                            JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot No command was recognized");
                        }
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 5500, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human Lost");
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the search of human");
                    JustinaHRI::stopFollowHuman();
                    JustinaHRI::enableLegFinder(false);
                    nextState=SM_MEMORIZING_OPERATOR;
                }        
                break;

            case SM_BRING_GROCERIES:
                std::cout << "State machine: SM_BRING_GROCERIES" << std::endl; 
                if(cont_z > 3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I am ready to take this to some location", 7000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Waiting for command to carry the bag");
                    cont_z=0;
                }
                cont_z++;
                if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
 
                    if(lastRecoSpeech.find("this bag to the bedroom") != std::string::npos){
                        location="bedroom";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the bed") != std::string::npos){
                        location="bed";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the desk") != std::string::npos){
                        location="desk";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the side table") != std::string::npos){
                        location="side_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the living room") != std::string::npos){
                        location="living_room";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the end table") != std::string::npos){
                        location="end_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the bookcase") != std::string::npos){
                        location="bookcase";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the couch") != std::string::npos){
                        location="couch";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the dining room") != std::string::npos){
                        location="dining_room";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the dining table") != std::string::npos){
                        location="dining_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the kitchen") != std::string::npos){
                        location="kitchen";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }


                    else if(lastRecoSpeech.find("this bag to the sink") != std::string::npos){
                        location="sink";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
 
                    else if(lastRecoSpeech.find("this bag to the dishwasher") != std::string::npos){
                        location="dishwasher";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
 
                    else if(lastRecoSpeech.find("this bag to the counter") != std::string::npos){
                        location="counter";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the storage table") != std::string::npos){
                        location="storage_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the cupboard") != std::string::npos){
                        location="cupboard";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the entrance") != std::string::npos){
                        location="entrance";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the corridor") != std::string::npos){
                        location="corridor";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "counter";
                        alig_to_place=true;
                        nextState = SM_BRING_GROCERIES_TAKE;
                    } 
                    if(location.compare("") != 0 && nextState == SM_BRING_GROCERIES_CONF){
                        ss.str("");
                        ss << "Do you want me take this bag to the "; 
                        tokens.clear();
                        boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                        for(int i = 0; i < tokens.size(); i++)
                            ss << tokens[i] << " ";

                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        
                        

                        JustinaHRI::waitAfterSay(ss.str(), 500, maxDelayAfterSay);
                        JustinaHRI::waitAfterSay("Tell me: yes or no", 500, maxDelayAfterSay);
                        
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                    }

                }
                break;

            case SM_BRING_GROCERIES_CONF:
                std::cout << "State machine: SM_BRING_GROCERIES_CONF" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                if(userConfirmation)
                    nextState = SM_BRING_GROCERIES_TAKE;
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_BRING_GROCERIES;
                    cont_z = 8;
                }
                else
                    nextState = SM_BRING_GROCERIES_TAKE;
                break;

            case SM_BRING_GROCERIES_TAKE:    
                std::cout << "State machine: SM_BRING_GROCERIES_TAKE" << std::endl;
               

                ss.str("");
                ss << "Ok , I will take it to the "; 
                tokens.clear();
                boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                for(int i = 0; i < tokens.size(); i++)
                    ss << tokens[i] << " ";
                ss << "then I will return to the car ,since I dont have a gripper, imagine I already have the bag";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                //JustinaManip::startTorsoGoTo(0.3, 0.0, 0.0);
                nextState=SM_BAG_DELIVERY;     
                break;

            case SM_BAG_DELIVERY:
                std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Command was recognized, carry the bag to: "+ location);
                
                if(!JustinaKnowledge::existKnownLocation(location)){
                    std::cout << "SM_BAG_DELIVERY: NO LOCATION!" << std::endl;
                    location="counter";//*************************************************************
                    alig_to_place=true;
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Location not found: "+ location);
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Change location to default location: counter ");
                }
                
                std::cout << "Location -> " << location << std::endl;
                if(!JustinaNavigation::getClose(location, 200000))
                    if(!JustinaNavigation::getClose(location, 200000))
                        JustinaNavigation::getClose(location, 200000);
                JustinaHRI::waitAfterSay("Here I am", 2000);
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Arrived to location: "+ location);
                nextState=SM_BAG_DELIVERY_PLACE;

                break;

            case SM_BAG_DELIVERY_PLACE:
                std::cout << "State machine: SM_BAG_DELIVERY_PLACE" << std::endl;
                JustinaHRI::waitAfterSay("I Arrived lets imagine I already gave you the bag", 3000);
                nextState=SM_LOOKING_HELP;
                break;

            case SM_LOOKING_HELP:
                std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
                
                JustinaHRI::waitAfterSay("I will look for help", 3000);
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Searching a human for help to the carry the bags");
                if(JustinaTasks::findPerson("", -1, JustinaTasks::STANDING, false, location)){
                    nextState=SM_GUIDING_ASK;
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Finish search, human was found");
                }
                else{
                    JustinaHRI::waitAfterSay("I did not find anyone", 3000); 
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Finish search, human was not found");
                }

                break;

            case SM_GUIDING_ASK:
                std::cout << "State machine: SM_GUIDING_ASK" << std::endl;
                JustinaHRI::waitAfterSay("Hey there can you please help me , please Tell me: yes or no", 1000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Waiting for human confirmation");
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                if(userConfirmation){
                    nextState = SM_GUIDING_MEMORIZING_OPERATOR_SAY;
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot yes command was recognized");
                }
                else {
                    nextState = SM_LOOKING_HELP;
                    JustinaNavigation::moveDistAngle(0.0, 1.5708, 2000);
                    JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot no command was recognized");
                }	    

                break;        

            case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
                JustinaHRI::waitAfterSay("I will guide you to the car", 4000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                JustinaHRI::waitAfterSay("Please, follow me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                location="entrance_door";
                if(!JustinaKnowledge::existKnownLocation(location)){
                    std::cout << "SM_BAG_DELIVERY: NO LOCATION!" << std::endl;
                    location="car_location";
                }
                cont_z=0;
                nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;

                break;

            case SM_GUIDING_MEMORIZING_OPERATOR_ELF:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_ELF" << std::endl;
                //JustinaHRI::enableLegFinderRear(true); ////igcdkjgdhghksd
                if(!JustinaNavigation::getClose(location, 200000))
                    if(!JustinaNavigation::getClose(location, 200000))
                        JustinaNavigation::getClose(location, 200000);
                JustinaHRI::waitAfterSay("Here I am", 2000);
                JustinaTools::pdfAppend("HelpMeCarry_Plans", "Arrived to location: "+ location);

                nextState = SM_FINAL_STATE;

                break;   

            case SM_FINAL_STATE:
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                JustinaTools::pdfStop("HelpMeCarry_Plans");
                success = true;
                break;

        }

        ros::spinOnce();
        loop.sleep();
    }



    return 1;
}

