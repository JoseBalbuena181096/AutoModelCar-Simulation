#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    speed = 0;
    steering = 0;
    speed_k = -400.0;
    steering_zero = 90;
    steering_k = 180/M_PI*2.7; //I don't know exactly where does this constant come from.
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    subCompressedImg   = n->subscribe("/app/camera/rgb/image_raw/compressed", 10, &QtRosNode::callbackCompressedImage, this);
    pubSpeed    = n->advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed"   , 10);
    pubSteering = n->advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", 10);

    int isZeroSpeedSent = 0;
    std_msgs::Int16 msgSpeed;
    std_msgs::Int16 msgSteering;
  
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        msgSpeed.data = (int)(speed*speed_k);
        if(speed == 0)
        {
            if(isZeroSpeedSent > 0)
            {
                pubSpeed.publish(msgSpeed);
                isZeroSpeedSent--;
            }
        }
        else
        {
            pubSpeed.publish(msgSpeed);
            isZeroSpeedSent = 5;
        }
        
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;   
}

void QtRosNode::callbackCompressedImage(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    msgCompressedImage = *msg;
    //std::cout << "camera img received with format " << msg->format << std::endl;
}

void QtRosNode::publishSteering(float steering)
{
    std_msgs::Int16 msgSteering;
    msgSteering.data = (int)(steering*steering_k + steering_zero);
    pubSteering.publish(msgSteering);
}
