#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "math.h"

using namespace std;
using namespace ros;

ros::Publisher publish_data;


int main(int argc, char **argv){
	ros::init(argc,argv,"xml_test");

    ros::NodeHandle private_nh("~");
	
	ros::NodeHandle nh;
	publish_data = nh.advertise<std_msgs::String>("talker",10);
    std::string para_val = private_nh.param<std::string>("demo_param","defaut v !");
    int int_para = private_nh.param<int>("int_param",7);
    Rate rate(10);
    while(ok()){
        std_msgs::String msg;
        msg.data = para_val;
        publish_data.publish(msg);
        for (int i=0; i<int_para; i++){
            printf("%d ",i);
        }
        printf("\n");
        spinOnce();
        rate.sleep();
    }
	

	ros::spin();

	return 0;
}