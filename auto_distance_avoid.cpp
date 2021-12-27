#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdlib.h"
#include "math.h"

#define CSR 2 //Closest Execptable Ranges

ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

int previous_index=0;

struct available_way{
	int begin;
	int end;
	int size;
};

available_way Check_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	available_way next;
	available_way now;
	next.size = 0;

	int status=0;	//0=nothing		1=previous are out of way
	int i;
	for (i=0;i<360;i++){
		if (status == 0){
			if (msg->ranges[i] > CSR) {
				now.begin=i;
				status=1;
			}
		}else {
			if (msg->ranges[i] < CSR) {
				now.end=i;
				status=0;
				now.size=now.end-now.begin;
				if (now.size > next.size) {
					next=now;
				}
			}
		}
	}
	printf ("from %d to %d is the next way, size of it is %d \n",next.begin,next.end,next.size);
	return next;
}



void Interup_function(const sensor_msgs::LaserScan::ConstPtr& msg){
	if (previous_index == 0){
		if ((msg->ranges[0] < 0.5)|(msg->ranges[20] < 0.5)|(msg->ranges[340] < 0.5) ){
			available_way return_way;
			return_way = Check_laser_scan(msg);

			int next_index=0;
			next_index = (return_way.begin + return_way.end)/2;
			previous_index = (3.1415*next_index)/(18);
			if (previous_index > 34) previous_index = 63 - previous_index;
		} else {
			data_msg.linear.x = 0.2;
			data_msg.angular.z = 0;
			publish_data.publish(data_msg);
		}
	} else {
		if (previous_index > 0){
			previous_index--;
			data_msg.linear.x = 0;
			data_msg.angular.z = -0.5;
		} else {
			previous_index++;
			data_msg.linear.x = 0;
			data_msg.angular.z = 0.5;
		}
		publish_data.publish(data_msg);
	}

}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::NodeHandle n;
	

	
	ros::Subscriber topic_sub = n.subscribe("scan",1000,Interup_function);

	ros::spin();

	return 0;
}