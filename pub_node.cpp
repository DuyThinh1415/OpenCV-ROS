#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
	ros::init(argc,argv,"publisher");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(1);
	geometry_msgs::Twist msg;
	msg.linear.x = 0.1;
	msg.angular.z = 2;


	while(ros::ok()){
		msg.linear.x  = msg.linear.x + 0.1;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
return 0;
}

