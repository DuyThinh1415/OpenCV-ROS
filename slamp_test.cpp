#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "demo_pakage/Num.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
using namespace cv;
using namespace std;

// ========================== Code part started ========================
#define pixel(f,i,c) (int)(*f.ptr(i,c))

class Coordinates{
    public:
    double x=0;
    double y=0;
    
    void set(double first, double sec){
        x=first;
        y=sec;
    }
};

Mat slamp_map = imread("/home/thinh/ROS/demo/src/demo_pakage/resource/full_black.png",IMREAD_COLOR);


class Data_Map{
    public:
    Coordinates newPoint;
    Coordinates oldPoint;
    float beta=0;   // RAD
    float old_vl=0;
    float old_va=0;

    float hpi=3.14159265358/2;

    void draw(){
        //printf(" \n Draw %5f : %5f \n",newPoint.x*100, newPoint.y*100);
        
        rectangle(slamp_map, Point(oldPoint.x*100-2,oldPoint.y*100-2), Point(oldPoint.x*100+2,oldPoint.y*100+2),Scalar(0,255,0),2,8,0);
        
        Mat slamp_output_layer = slamp_map.clone();
        rectangle(slamp_output_layer, Point(newPoint.x*100-2,newPoint.y*100-2), Point(newPoint.x*100+2,newPoint.y*100+2),Scalar(0,0,255),2,8,0);
    
        rectangle(slamp_output_layer, Point((newPoint.x + 0.2*sin(beta))*100-1,(newPoint.y + 0.2*cos(beta))*100-1), Point((newPoint.x + 0.2*sin(beta))*100+1,(newPoint.y + 0.2*cos(beta))*100+1),Scalar(0,0,255),2,8,0);
    
        imshow("map",slamp_output_layer);
    }

    Data_Map(){
        newPoint.set(2,1);
        oldPoint.set(2,1);
    }

    // WARNING, do not read the following code, your brain will being fuck in to the ass !!!
    void update_nhan(float vl, float va, float t){
        //printf("%5f - %5f - %5f \n",vl, va, t);

        double delta_x;
        double delta_y;

        if (old_va != 0){
            
            delta_x=(-1*old_vl/old_va)*((cos(beta+old_va*t) - cos(beta)));
            delta_y=(   old_vl/old_va)*((sin(beta+old_va*t) - sin(beta)));
            
        } else {
            delta_x=old_vl*t*cos(hpi - beta);
            delta_y=old_vl*t*cos(beta);
        }
        //printf("delta_x: %5f\n",delta_x);
        //printf("delta_y: %5f\n",delta_y);
        //cap nhat vi tri robot

        //cap nhat beta
        beta = beta + t*old_va;
        //printf("beta: %5f\n",beta);

        oldPoint=newPoint;
        newPoint.x+=delta_x;
        newPoint.y+=delta_y;
        //printf("new.x: %5f\n",newPoint.x);
        draw();

        //cap nhat gia tri van toc cho lan update tiep theo
        old_va=va;
        old_vl=vl;
    }

    void update_slamp(float t, const sensor_msgs::LaserScan::ConstPtr& msg){
        update_nhan(old_vl, old_va, t);
        int each;
        for (each=0; each < 360; each++){
            if (!isinf(msg->ranges[each])){
                float alpha=beta + 4*hpi*each/360;
                Coordinates dest;
                dest.x = newPoint.x + msg->ranges[each]*sin(alpha);
                dest.y = newPoint.y + msg->ranges[each]*cos(alpha);

                dest.x*=100;
                dest.y*=100;
                int p = slamp_map.at<cv::Vec3b>(dest.y, dest.x)[0];
                int write_val = 0;
                //printf("%5f  -  %5f \n",dest.x, dest.y);
                if (p > 250) continue;
                    else write_val=p+5;
                rectangle(slamp_map, Point(dest.x+1,dest.y+1), Point(dest.x-1,dest.y-1),Scalar(write_val,0,100),2,2,0);
        
            }
        }
        printf("\n");
    }
};

Data_Map data_map;

int cout_bla=0;
double now_time=0;
double pre_time=0;

void velo_recive(const geometry_msgs::Twist::ConstPtr& msg){
    double find_time=ros::Time::now().toSec();
	//printf("%d data recive \n",cout_bla++);
    pre_time = now_time;
    now_time = ros::Time::now().toSec();
    data_map.update_nhan(msg->linear.x, msg->angular.z, now_time-pre_time);
    

    char c=(char)waitKey(3);
    if(c==27){
    	printf("\n ===> Sutdown Signal Recive <=== \n");
		cv::destroyAllWindows();
		ros::shutdown();
    }
    printf("running time : %7f \n",ros::Time::now().toSec()-find_time);
}

void scan_recive(const sensor_msgs::LaserScan::ConstPtr& msg){
    pre_time = now_time;
    now_time = ros::Time::now().toSec();
    data_map.update_slamp(now_time-pre_time, msg);
    imshow("map",slamp_map);
}

void Init(){
    printf("INIT \n");
    resize(slamp_map, slamp_map, Size(512,512), 0, 0, CV_INTER_LINEAR);
    while (ros::Time::now().toSec() == 0);
    printf("complete INIT \n");
}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	ros::NodeHandle n;
	ros::Subscriber sub_velo = n.subscribe("/cmd_vel",10,velo_recive);

    ros::NodeHandle nh;
    ros::Subscriber sub_lidar = nh.subscribe("/scan",10,scan_recive);

    Init();
	ros::spin();

	return 0;
}