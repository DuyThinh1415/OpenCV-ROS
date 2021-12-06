#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdlib.h"

ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

int simple_count=0;

void writeMsgToLog(const sensor_msgs::LaserScan::ConstPtr& msg){

	if (simple_count < 0){
		simple_count++;
		data_msg.linear.x = 0;
		data_msg.angular.z = 1;
		printf("Turn left ! \n %f %f %f :: %f %f",msg->ranges[0],msg->ranges[345],msg->ranges[15],msg->ranges[60],msg->ranges[300]);
		publish_data.publish(data_msg);
		return;
	} else if (simple_count > 0){
		simple_count--;
		data_msg.linear.x = 0;
		data_msg.angular.z = -1;
		printf("Turn right ! \n %f %f %f :: %f %f",msg->ranges[0],msg->ranges[345],msg->ranges[15],msg->ranges[60],msg->ranges[300]);
		publish_data.publish(data_msg);
		return;
	}

	data_msg.linear.x = 0.2;
	data_msg.angular.z = 0;

	if ((msg->ranges[0] < 0.5)|(msg->ranges[15] < 0.5)|(msg->ranges[345] < 0.5) ){
			if (msg->ranges[60] > msg->ranges[300]){
				data_msg.linear.x = 0;
				data_msg.angular.z = 1;
				printf("Turn left first time ! \n %f %f %f :: %f %f",msg->ranges[0],msg->ranges[345],msg->ranges[15],msg->ranges[60],msg->ranges[300]);
				simple_count=-5;

			} else {
				data_msg.linear.x = 0;
				data_msg.angular.z = -1;
				printf("Turn right first time ! \n %f %f %f :: %f %f",msg->ranges[0],msg->ranges[345],msg->ranges[15],msg->ranges[60],msg->ranges[300]);
				simple_count=5;
			}
	} else  printf("move forward normaly ! \n %f %f %f :: %f %f",msg->ranges[0],msg->ranges[345],msg->ranges[15],msg->ranges[60],msg->ranges[300]);

	publish_data.publish(data_msg);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::NodeHandle n;
	

	
	ros::Subscriber topic_sub = n.subscribe("scan",1000,writeMsgToLog);

	ros::spin();

	return 0;
}