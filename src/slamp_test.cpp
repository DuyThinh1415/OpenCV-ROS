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

class Coordinates{
    public:
    float x=0;
    float y=0;
    
    void set(float first, float sec){
        x=first;
        y=sec;
    }
};

Mat odom_map = imread("/home/thinh/ROS/demo/src/demo_pakage/src/full_black.png",IMREAD_COLOR);

class Data_Map{
    public:
    Coordinates newPoint;
    Coordinates oldPoint;
    float beta=0;   // RAD
    float old_vl=0;
    float old_va=0;

    float hpi=3.14159265358/2;

    void draw(){
        printf(" \n Draw %5f : %5f \n",newPoint.x*100, newPoint.y*100);
        rectangle(odom_map, Point(oldPoint.x*100-2,oldPoint.y*100-2), Point(oldPoint.x*100+2,oldPoint.y*100+2),Scalar(0,255,0),2,8,0);
        rectangle(odom_map, Point(newPoint.x*100-2,newPoint.y*100-2), Point(newPoint.x*100+2,newPoint.y*100+2),Scalar(0,0,255),2,8,0);
    
        rectangle(odom_map, Point((newPoint.x + 0.2*sin(beta))*100-1,(newPoint.y + 0.2*cos(beta))*100-1), Point((newPoint.x + 0.2*sin(beta))*100+1,(newPoint.y + 0.2*cos(beta))*100+1),Scalar(0,0,255),2,8,0);
    
    }

    Data_Map(){
        newPoint.set(2,1);
        oldPoint.set(2,1);
    }

    // WARNING, do not read the following code, your brain will being fuck in to the ass !!!
    void update(float vl, float va, float t){
        printf("%5f - %5f - %5f \n",vl, va, t);

        float delta_x;
        float delta_y;

        if (old_va != 0){
            //alpha
            float alpha = old_va*t;
            printf("alpha: %5f\n",alpha);
            //r la ban kinh duong tron
            float r=old_vl/old_va;
            printf("r: %5f\n",r);
            // do chenh lech cua hoanh do
            delta_x = r*(1-cos(alpha));
            printf("delta_x: %5f\n",delta_x);
            //do chenh lech cua tung do
            delta_y=r*sin(alpha);
            printf("delta_y: %5f\n",delta_y);

            //Tinh do dai di chuyen
            float s=sqrt(delta_x*delta_x + delta_y*delta_y);
            
            //Xoay he toa do lai cho dung voi vi tri cua no
            delta_x-=s*(cos(hpi - alpha/2 + beta) - cos(hpi - alpha/2));
            delta_y-=s*(cos(alpha/2) - cos(alpha/2 + beta));
            
        } else {
            delta_x=old_vl*t*cos(hpi - beta);
            delta_y=old_vl*t*cos(beta);
        }
        printf("delta_x: %5f\n",delta_x);
        printf("delta_y: %5f\n",delta_y);
        //cap nhat vi tri robot

        //cap nhat beta
        beta = beta + t*old_va;
        printf("beta: %5f\n",beta);

        oldPoint=newPoint;
        printf("old.x: %5f\n",newPoint.x);
        newPoint.x+=delta_x;
        newPoint.y+=delta_y;
        printf("new.x: %5f\n",newPoint.x);
        draw();

        //cap nhat gia tri van toc cho lan update tiep theo
        old_va=va;
        old_vl=vl;
    }

    void update_odom(float t, const sensor_msgs::LaserScan::ConstPtr& msg){
        update(old_vl, old_va, t);
        int each;
        for (each=0; each < 360; each++){
            if (!isinf(msg->ranges[each])){
                float alpha=beta + 4*hpi*each/360;
                Coordinates dest;
                dest.x = newPoint.x + msg->ranges[each]*sin(alpha);
                dest.y = newPoint.y + msg->ranges[each]*cos(alpha);

                dest.x*=100;
                dest.y*=100;
                
                rectangle(odom_map, Point(dest.x+1,dest.y+1), Point(dest.x-1,dest.y-1),Scalar(255,0,0),2,2,0);
        
            }
        }
    }
};

Data_Map data_map;

int cout_bla=0;
double now_time=0;
double pre_time=0;

void velo_recive(const geometry_msgs::Twist::ConstPtr& msg){
	printf("%d data recive \n",cout_bla++);
    pre_time = now_time;
    now_time = ros::Time::now().toSec();
    data_map.update(msg->linear.x, msg->angular.z, now_time-pre_time);
    imshow("map",odom_map);

    char c=(char)waitKey(3);
    if(c==27){
    	printf("\n ===> Sutdown Signal Recive <=== \n");
		cv::destroyAllWindows();
		ros::shutdown();
    }
    printf("\n");
}

void scan_recive(const sensor_msgs::LaserScan::ConstPtr& msg){
    pre_time = now_time;
    now_time = ros::Time::now().toSec();
    data_map.update_odom(now_time-pre_time, msg);
    imshow("map",odom_map);
}

void Init(){
    printf("INIT \n");
    resize(odom_map, odom_map, Size(512,512), 0, 0, CV_INTER_LINEAR);
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