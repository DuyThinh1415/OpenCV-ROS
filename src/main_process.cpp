#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "demo_pakage/Num.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "main_process.h"
#include "main_process_lib.h"
 
using namespace cv;
using namespace std;


void Init_system(void){
    printf("System Initialize... \n");

    ros::NodeHandle private_nh("~");
    int verify=private_nh.param<float>("verify",0);
    if (!verify) printf(" => WARNING: the yaml file is unreadable !!! \n");

    para.acceleration_max = private_nh.param<float>("acceleration_max",0.05);
    para.acceleration_ratio = private_nh.param<float>("acceleration_ratio",0.4);

    while (ros::Time::now().toSec() == 0);
    ram.now_time=ros::Time::now().toSec();

    printf("System Initialize Complete \n");
}

int main(int argc, char **argv){
	ros::init(argc,argv,"lane_detection");
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	ros::NodeHandle get_image;
	//ros::Subscriber topic_sub = get_image.subscribe("/cv_camera/image_raw",1000,image_recive);
    ros::Subscriber topic_sub = get_image.subscribe("/camera/rgb/image_raw",1000,image_recive);

    ros::NodeHandle contro_sig;
	ros::Subscriber another_topic_sub = contro_sig.subscribe("/controller_topic",1000,contro_sig_recive);


    Init_system();
	ros::spin();

	return 0;
}
