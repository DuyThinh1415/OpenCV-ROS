#include "ros/ros.h"
#include "std_msgs/String.h"
#include "demo_pakage/Num.h"
#include "geometry_msgs/Twist.h"

#include "math.h"
#include "stdlib.h"

void ReturnFunc(const sensor_msgs::Twist::ConstPtr& msg){
    
}

int main(int argc, char **argv){
	ros::init(argc,argv,"processor");
	ros::NodeHandle n;
    ros::NodeHandle nh;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(1);
	geometry_msgs::Twist msg;

    ros::Subscriber topic_sub = nh.subscribe("lane_vel",1000,ReturnFunc);
return 0;
}