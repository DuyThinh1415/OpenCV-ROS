#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "vector"

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
#define r_zone 20

class Coordinates{
    public:
    double x=0;
    double y=0;
    
    void set(double first, double sec){
        x=first;
        y=sec;
    }
};

Mat full_black_mat = imread("/home/thinh/ROS/demo/src/demo_pakage/resource/full_black.png",IMREAD_COLOR);

Mat slamp_output_layer;

class One_pixel_in4{
    public:
    float x=0;
    float y=0;
    int type=0; //  0 = unknow, 1=new, 2=normall, 3= old
    int scan=0; // did this pixel scan yet ?
    int label=0;
    int index_of_conected_pixel = -1;
};

vector<One_pixel_in4> global_list;
vector<One_pixel_in4> tmp_for_rec;
vector<One_pixel_in4> labeled_list;

class Lidar_frame{
    public:
    Mat local_map;
    Lidar_frame(void){
        local_map = full_black_mat.clone();
        cvtColor(local_map, local_map, COLOR_RGB2GRAY);
    }
    One_pixel_in4 pixel_list[360];
    int size=0;

    void clear_data(void){
        local_map = full_black_mat.clone();
        cvtColor(local_map, local_map, COLOR_RGB2GRAY);
        for (int i=0; i<360; i++) pixel_list[i].scan=0;
        size=0;
    }

    void set(Coordinates input_cord){
        pixel_list[size].x = input_cord.x;
        pixel_list[size].y = input_cord.y;
        local_map.at<char>(input_cord.x,input_cord.y)=255;
        size++;
        if (size > 360) printf("\n WARNING: Lidar_frame.size out of range !!! \n");
    }
};

int get_manhatan_distance(One_pixel_in4 one, One_pixel_in4 two){
    return abs(one.x - two.x)+abs(one.y-two.y);
}

void rec_scan(int group_value){
    while(tmp_for_rec.size() != 0){
        int check_index = tmp_for_rec.size() -1;
        for (int i=global_list.size()-1; i>0; i--){
            if (get_manhatan_distance(global_list[i],tmp_for_rec[check_index]) < r_zone){
                tmp_for_rec.push_back(global_list[i]);
                global_list.erase(global_list.begin()+i);
            }
        }
        tmp_for_rec[check_index].label=group_value;
        labeled_list.push_back(tmp_for_rec[check_index]);
        tmp_for_rec.erase(tmp_for_rec.begin() + check_index);
    }

}

void movement_predict_process(Lidar_frame nframe, Lidar_frame oframe){
    //printf(" => Go to movement_predict_process \n");
    labeled_list.clear();

    int i;
    for (i=0; i<nframe.size; i++){
        if (oframe.local_map.at<char>(nframe.pixel_list[i].x,nframe.pixel_list[i].y) == 0){
            nframe.pixel_list[i].label = 1;
            global_list.push_back(nframe.pixel_list[i]);
        }
    }
    //printf(" => Go to second movement_predict_process \n");
    for (i=0; i<oframe.size; i++){        
        if (nframe.local_map.at<char>(oframe.pixel_list[i].x, oframe.pixel_list[i].y) == 0){
            oframe.pixel_list[i].label=3;
            global_list.push_back(oframe.pixel_list[i]);
        }
    }

    int number_of_group=0;
    printf(" Size of global list : %d \n",(int)global_list.size());

    while(global_list.size() > 0){
        tmp_for_rec.push_back(global_list[global_list.size()-1]);
        global_list.pop_back();
        rec_scan(number_of_group);
        number_of_group++;
    }
    printf(" Number of group : %d \n",number_of_group);
    //printf(" => Exit movement_predict_process \n");
    
}

class Data_Map{
    public:
    Coordinates newPoint;
    Coordinates oldPoint;

    Lidar_frame oldFrame;
    Lidar_frame newFrame;

    Data_Map(){
        newPoint.set(2,1);
        oldPoint.set(2,1);
    }

    float beta=0;   // RAD
    float old_vl=0;
    float old_va=0;
    float hpi=3.14159265358/2;

    void draw(){
        //printf(" \n Draw %5f : %5f \n",newPoint.x*100, newPoint.y*100);
        
        rectangle(slamp_output_layer, Point(oldPoint.x*100-2,oldPoint.y*100-2), Point(oldPoint.x*100+2,oldPoint.y*100+2),Scalar(0,255,0),2,8,0);
        
        
        rectangle(slamp_output_layer, Point(newPoint.x*100-2,newPoint.y*100-2), Point(newPoint.x*100+2,newPoint.y*100+2),Scalar(0,0,255),2,8,0);
        
        if (labeled_list.size() != 0){

            int maxx=labeled_list[0].x;
            int maxy=labeled_list[0].y;
            int minx=labeled_list[0].x;
            int miny=labeled_list[0].y;
            int old_label=0;
        
            for (int i=1; i<labeled_list.size();i++){
                if (old_label == labeled_list[i].label){
                    if (labeled_list[i].x > maxx) maxx = labeled_list[i].x;
                    if (labeled_list[i].x < minx) minx = labeled_list[i].x;
                    if (labeled_list[i].y > maxy) maxy = labeled_list[i].y;
                    if (labeled_list[i].y < miny) miny = labeled_list[i].y;
                } else {
                    rectangle(slamp_output_layer, Point(maxx+2,maxy+2), Point(minx-2,miny-2),Scalar(120,70,200),2,8,0);
                    
                    old_label = labeled_list[i].label;

                    maxx=labeled_list[i].x;
                    maxy=labeled_list[i].y;
                    minx=labeled_list[i].x;
                    miny=labeled_list[i].y;
                }
            }
        }
        rectangle(slamp_output_layer, Point((newPoint.x + 0.2*sin(beta))*100-1,(newPoint.y + 0.2*cos(beta))*100-1), Point((newPoint.x + 0.2*sin(beta))*100+1,(newPoint.y + 0.2*cos(beta))*100+1),Scalar(0,0,255),2,8,0);
    
        imshow("map",slamp_output_layer);
        slamp_output_layer = full_black_mat.clone();
        char c=(char)waitKey(3);
        if(c==27){
            printf("\n ===> Sutdown Signal Recive <=== \n");
            cv::destroyAllWindows();
            ros::shutdown();
        }
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

        oldFrame = newFrame;
        newFrame.clear_data();

        int each;
        for (each=0; each < 360; each++){
            if (!isinf(msg->ranges[each])){
                float alpha=beta + 4*hpi*each/360;
                Coordinates dest;
                dest.x = newPoint.x + msg->ranges[each]*sin(alpha);
                dest.y = newPoint.y + msg->ranges[each]*cos(alpha);

                dest.x*=100;
                dest.y*=100;
                if ((dest.x > 0) & (dest.x < 512) & (dest.y > 0) & (dest.y < 512)){
                    int write_val = 255;
                    rectangle(slamp_output_layer, Point(dest.x+1,dest.y+1), Point(dest.x-1,dest.y-1),Scalar(write_val,0,100),2,2,0);
                    newFrame.set(dest);
                }
                
            }
        }
        movement_predict_process(newFrame, oldFrame);
        printf("\n");
    }

};

Data_Map data_map;

int cout_bla=0;
double now_time=0;
double pre_time=0;

void velo_recive(const geometry_msgs::Twist::ConstPtr& msg){
    printf("1");
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
    printf("2");
}

void scan_recive(const sensor_msgs::LaserScan::ConstPtr& msg){
    printf("3");
    double find_time=ros::Time::now().toSec();
    pre_time = now_time;
    now_time = ros::Time::now().toSec();
    data_map.update_slamp(now_time-pre_time, msg);
    //imshow("map",slamp_map);
    printf("running time : %7f \n",ros::Time::now().toSec()-find_time);
    printf("4");
}

void Init(){
    printf("INIT \n");
    resize(full_black_mat, full_black_mat, Size(512,512), 0, 0, CV_INTER_LINEAR);
    slamp_output_layer = full_black_mat.clone();
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