#include "ros/ros.h"
#include "std_msgs/String.h"
#include "demo_pakage/Num.h"

#include "math.h"
#include "stdlib.h"

demo_pakage::Num encode(char* data){
	demo_pakage::Num signal;

	return signal;

}

int main(int argc, char **argv){
	ros::init(argc,argv,"publisher");
	ros::NodeHandle n;

	ros::Publisher controller_pub = n.advertise<std_msgs::String>("controller_topic", 1000);
	demo_pakage::Num control_signal;
	ros::Rate loop_rate(1);

	char data[100];

	while(ros::ok()){
		std::cin.getline(data,100);
		if (data[0] == '*') {
			printf("Shutdown signal !");
			break;
		}
		control_signal = encode(data);
		controller_pub.publish(control_signal);
		ros::spinOnce();
		if (data[0] == '*') break;
	}

	

return 0;
}
