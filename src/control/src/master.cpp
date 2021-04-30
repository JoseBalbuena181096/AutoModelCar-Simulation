#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include<math.h>

using namespace std;

enum{LINE,OBJECT,PARKING} state = LINE;  
bool on_off = false;

std_msgs::Int16 angle_message;
std_msgs::Int16 speed_message;

int angle_pid = 0;
int angle_now = 0;
int angle_last = 0;
float kp_angle = 1.025;//1.025
float kd_angle = 0.01085;
float error_angle = 0.0;
int u_angle = 0;

int speed_pid = 0;
float kp_speed = 1.025;
int u_speed = 0;

void angle_nowCallback(const std_msgs::Int16& angle_now_message){
    angle_pid = static_cast<int>(angle_now_message.data);
    u_angle = static_cast<int>(kp_angle*static_cast<float>(angle_pid) + kd_angle*static_cast<float>(angle_pid - angle_last));
    angle_pid = 90 + u_angle;
    if(angle_pid<= 40)
	    angle_pid = 40;
    else if(angle_pid >= 140)
	    angle_pid = 140;
    u_speed = static_cast<int>(kp_speed * angle_pid);
    speed_pid = - 800 + abs(u_speed);
    //speed_pid = -200;
    if(speed_pid < - 800)
	    speed_pid = - 800;
    else if(speed_pid > -20)
	    speed_pid = -20;
    angle_last = angle_pid;
		  
}
void on_offCallback(const std_msgs::Int16& start_message){
    if(start_message.data == 1)
      on_off = true;
    else if(start_message.data == 0)
      on_off = false;
}

int main(int argc, char **argv){
    ros::init(argc, argv,"Master");   
    ros::NodeHandle nh;   
    //ros::Subscriber angle_now = nh.subscribe("/angle_line_now",1,angle_nowCallback);
    ros::Subscriber angle_now = nh.subscribe("/distance_center_line",1,angle_nowCallback);
    ros::Subscriber on_off_sub = nh.subscribe("/on_off",1,on_offCallback);
    ros::Publisher angle_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering",1000);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed",1000);
    ros::Rate loop_rate(40);   
  int count = 0;
  while (ros::ok())
  {
    if(on_off){
      if (state == LINE){
          angle_message.data = angle_pid;
          angle_pub.publish(angle_message);
          speed_message.data = speed_pid;
          speed_pub.publish(speed_message);
        }
    } 
    else{
        angle_message.data = 0;
        angle_pub.publish(angle_message);
        speed_message.data = 0;
        speed_pub.publish(speed_message);
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
