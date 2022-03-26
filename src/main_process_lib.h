#ifdef MAIN_PROCESS_LIB_H

#define MAIN_PROCESS_LIB_H

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

using namespace cv;
using namespace std;

class Ram{
  public:
  int     CSR=15;
  int     LRS=25; // number of point get  ( LRS < 50)
  int     RLS=40; // Range of Lane search
  int     sample_jump=5;
  float   Speed=0.1;

//==================== You shouldn't change any variables below ! ===========================

  int     frame_count=0;
  float   final_angular_velo=0;
  int     FSM_state=0;
  int     counter_state_1=0;
  float   output_speed = 0;
  int     lane_follow = 1;  // 1= center, 2=left, 3=right

  float   change_lane_V_angular;
  float   change_lane_clk1;
  float   change_lane_alpha;
  float   change_lane_b;
  float   change_lane_d1;
  int     change_lane_direction;
  float   change_lane_clk2;
  float   change_lane_remaning_S;

  float   Now_FPS;
  float   counter_FPS=0;
  double  now_time;
  double  previous_time;

};

class Para{
  public:
  float acceleration_max=0;
  float acceleration_ratio=0;
};

Para para;

Ram ram;
demo_pakage::Num present_command;
ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

class Lane{
public:
  int col[50]={0}; //col index
  int row[50]={0}; //row index
  bool trust[50]={0};

  void checkCSR(void){
    for(int i=1; i<ram.LRS-1;i++){
      float tmp=(col[i-1]+col[i+1])/2.0-col[i];
      //printf("check %d: %d and %d => %d \n",i,col[i],trust[i],((tmp*tmp < CSR) & trust[i]));
      trust[i]=((tmp*tmp < ram.CSR) & trust[i]);
    }
  }

};

Mat get_Trainform_matrix();

float process(Mat frame);

cv_bridge::CvImagePtr convert(const sensor_msgs::Image::ConstPtr& msg);

int change_lane_process();

void contro_sig_recive(const demo_pakage::Num msg);

void image_recive(const sensor_msgs::Image::ConstPtr& msg);

#endif 


