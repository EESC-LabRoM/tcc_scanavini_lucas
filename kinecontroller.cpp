#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

std_msgs::Float32 Wr;
std_msgs::Float32 Wl;
ros::Publisher R_Motor;
ros::Publisher L_Motor; 

void callback(const geometry_msgs::TwistConstPtr& vel)
{               
        Wr.data = (vel->linear.x)/0.35 + (vel->angular.z)/2 ;
        Wl.data = (vel->linear.x)/0.35 - (vel->angular.z)/2 ;
        R_Motor.publish(Wr);
        L_Motor.publish(Wl);        
       
}

int main( int argc, char **argv)
{
    ros::init(argc,argv, "Kinect_Controller");
    
    ros::NodeHandle node;
    
    R_Motor = node.advertise<std_msgs::Float32>("/AMR/motorRightSpeed" , 1);   
    L_Motor = node.advertise<std_msgs::Float32>("/AMR/motorLeftSpeed" , 1 );
    ros::Subscriber s = node.subscribe("/cmd_vel",1, callback); 
    ros::spin();
}   