#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "math.h"

#include "stdlib.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

int count=0;
double nowtime, pretime;

void When_get_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	//printf("					Laser recived ! => %d \n",count++);
	
}

int tmp=0;
double fps=0;

void When_get_image(const sensor_msgs::Image::ConstPtr& msg){
	printf("	Camera recived !   %d \n",count++);
	pretime=nowtime;
	nowtime =ros::Time::now().toSec();
	printf("time : %5f \n",nowtime-pretime);
	tmp++;
	fps += 1.0/(nowtime-pretime);
	printf("FPS predict : %5f \n",fps/tmp);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::NodeHandle n;
	ros::NodeHandle n2;
	

	
	ros::Subscriber topic_sub = n.subscribe("scan",1000,When_get_scan);
	ros::Subscriber topic_sub_2 = n2.subscribe("/camera/rgb/image_raw",1000,When_get_image);
	while(ros::Time::now().toSec() == 0){
		printf("Waitting... \n");
	};
	ros::spin();

	return 0;
}