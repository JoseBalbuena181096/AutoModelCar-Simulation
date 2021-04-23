#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Int16.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();
    
    float speed;    //Speed is intended to be given in m/s
    float steering; //Steering is intended to be given in rad
    float speed_k;
    int   steering_zero;
    float steering_k;
    sensor_msgs::CompressedImage msgCompressedImage;

    ros::NodeHandle* n;
    ros::Subscriber subCompressedImg;
    ros::Publisher  pubSpeed;
    ros::Publisher  pubSteering;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publishSteering(float steering);
    void callbackCompressedImage(const sensor_msgs::CompressedImage::ConstPtr& msg);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
