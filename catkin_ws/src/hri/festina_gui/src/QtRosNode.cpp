#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    linearSpeed= 0;
    angularSpeed = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    int isZeroSpeedSent = 0;
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        loop.sleep();
        ros::spinOnce();
    if(linearSpeed == 0 && angularSpeed == 0)
        {
            if(isZeroSpeedSent > 0)
            {
                JustinaHardware::setBaseCmdVel(0,0,0);
                isZeroSpeedSent--;
            }
        }
        else
        {
            JustinaHardware::setBaseCmdVel(linearSpeed,0,angularSpeed);
            isZeroSpeedSent = 5;
        }
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    JustinaHardware::setNodeHandle(nh);
    JustinaNavigation::setNodeHandle(nh);
    JustinaHRI::setNodeHandle(nh);
    JustinaVision::setNodeHandle(nh);
}