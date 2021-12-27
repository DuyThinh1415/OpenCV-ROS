#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

void writeMsgToLog(const sensor_msgs::LaserScan::ConstPtr& msg){
	if (isinf(msg->ranges[0])) ROS_INFO("infiny detected \n");

}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("new_cmd_vel",1000);
	ros::NodeHandle n;
	

	
	ros::Subscriber topic_sub = n.subscribe("new_topic_scan",1000,writeMsgToLog);

	ros::spin();

	return 0;
}