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
 
using namespace cv;
using namespace std;

/*
This file is made by Duy Thinh
this is version 1.5.2.1
last update: 31/12/2021
*/

/*
                              _
                           _ooOoo_
                          o8888888o
                          88" . "88
                          (| -_- |)
                          O\  =  /O
                       ____/`---'\____
                     .'  \\|     |//  `.
                    /  \\|||  :  |||//  \
                   /  _||||| -:- |||||_  \
                   |   | \\\  -  /'| |   |
                   | \_|  `\`---'//  |_/ |
                   \  .-\__ `-. -'__/-.  /
                 ___`. .'  /--.--\  `. .'___
              ."" '<  `.___\_<|>_/___.' _> \"".
             | | :  `- \`. ;`. _/; .'/ /  .' ; |
             \  \ `-.   \_\_`. _.'_/_/  -' _.' /
   ===========`-.`___`-.__\ \___  /__.-'_.'_.-'================
                           `=--=-'         
   
   Lạy chúa trên cao, xin chúa phù hộ code con chạy không bug
   nam mô a di đà phật
*/

//Define below use for debug prupose, only yes or no.
#define debug_change_lane 0
#define debug_process_image 1

// ========================== Code part started ========================
#define pixel(f,i,c) (int)(*f.ptr(i,c))

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

  // ============== Parameter ========================

  float acceleration_max=0.05;
  float acceleration_ratio=0.3;
};

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

Mat get_Trainform_matrix(){
  Point2f src_p[4];
  Point2f dst_p[4];

  src_p[0]=Point2f(470.0f, 0.0f);
  src_p[1]=Point2f(640.0f, 150.0f);
  src_p[2]=Point2f(0.0f, 150.0f);
  src_p[3]=Point2f(170.0f, 0.0f);

  dst_p[0]=Point2f(1280.0f, 0.0f);
  dst_p[1]=Point2f(512.0f, 768.0f);
  dst_p[2]=Point2f(256.0f, 768.0f);
  dst_p[3]=Point2f(-512.0f, 0.0f);

  Mat trans_matrix=getPerspectiveTransform(src_p, dst_p);
  return trans_matrix;
 }

float process(Mat frame){
  Mat gray;
  cvtColor(frame, gray, COLOR_RGB2GRAY);
  Mat crop = gray(Range(240,480),Range(0,640));
  Mat warp;
  warp.create(crop.size(), crop.type());
  warpPerspective(crop, warp, get_Trainform_matrix(),Size(768,768),INTER_LINEAR);
  warp=warp(Range(256,768),Range(0,768));
  GaussianBlur(warp, warp, Size(5,5), 0);

  //threshold(warp,warp,127,255, THRESH_BINARY); 
  //Canny(warp,warp,100,255);
  adaptiveThreshold(warp,warp,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,11,5);

  line(warp, Point(6,408), Point(253,514),Scalar(0),7,8,0);
  line(warp, Point(757,410), Point(518,512),Scalar(0),7,8,0);

  int find_started=0;
  Mat cut_for_sum;

  for (int i=448;i<512;i++){
      find_started += (int)(*warp.ptr(i,384));
    }
  if (find_started == 0 ) cut_for_sum=warp(Range(448,512),Range(128,640));
  else cut_for_sum=warp(Range(320,384),Range(128,640));
  
  Mat frame_for_draw;
  cvtColor(warp, frame_for_draw, COLOR_GRAY2RGB);

  frame = warp;  //512*768
                // cut for sum :128*512

  int center=256;
  
  while (center < 512){
    int tmp=0;
    for (int i=0;i<64;i++){
      tmp=tmp+(int)(*cut_for_sum.ptr(i,center));
    }
    if (tmp > 2000) break;
    center++;
  }
  int right_start=center+128;

  center=255;
  while (center >0){
    int tmp=0;
    for (int i=0;i<64;i++){
      tmp=tmp+(int)(*cut_for_sum.ptr(i,center));
    }
    if (tmp > 2000) break;
    center--;
  }

  int left_start=center+128;
  if (debug_process_image){
    Point p1(left_start,0), p2(left_start,512);
    line(frame_for_draw, p1, p2, Scalar(255,0,0), 2, LINE_4);
    p1=Point(right_start,0);
    p2=Point(right_start,512);
    line(frame_for_draw, p1, p2, Scalar(255,0,0), 2, LINE_4);
  }
  //================================ detect started ===============================

  Lane left, right, mid, trust;

  int count=0;
  int check_row=500;
  // Left check
  while (check_row > 500-ram.LRS*ram.sample_jump){
    for (int i=left_start+ram.RLS; i>left_start-ram.RLS; i--){
      if (pixel(frame,check_row,i) != 0){
        left.col[count]=i;
        left.trust[count]=1;
        //rectangle(frame_for_draw, Point(i+1, check_row+1), Point(i-1,check_row-1),Scalar(0,0,255),2,8,0);
        if (i != left_start+ram.RLS) left_start=i;
        break;
      }
    }
    left.row[count]=check_row;
    count++;
    check_row-=ram.sample_jump;
  }

  count=0;
  check_row=500;

  while (check_row > 500-ram.LRS*ram.sample_jump){
    for (int i=right_start-ram.RLS; i<right_start+ram.RLS; i++){
      if (pixel(frame,check_row,i) != 0){
        right.col[count]=i;
        right.trust[count]=1;
        //rectangle(frame_for_draw, Point(i+1, check_row+1), Point(i-1,check_row-1),Scalar(0,100,255),2,8,0);
        if (i != right_start-ram.RLS)right_start=i;
        break;
      }
    }

    right.row[count]=check_row;
    count++;
    check_row-=ram.sample_jump;
  }
  
  //================================ CSR check ===============================
  left.checkCSR();
  right.checkCSR();

  for (int i=0; i<ram.LRS; i++){
    mid.row[i]=left.row[i];
    mid.col[i]=(left.col[i] + right.col[i])/2;
    mid.trust[i]=(left.trust[i] & right.trust[i]);
  }

  if (debug_process_image){
    for (int i=0; i<ram.LRS; i++){
      if (left.trust[i]){
        rectangle(frame_for_draw, Point(left.col[i]+1,left.row[i]+1), Point(left.col[i]-1,left.row[i]-1),Scalar(0,0,255),2,8,0);
      }
      else {
        rectangle(frame_for_draw, Point(left.col[i]+1,left.row[i]+1), Point(left.col[i]-1,left.row[i]-1),Scalar(0,255,0),2,8,0);
      }

      if (right.trust[i]){
        rectangle(frame_for_draw, Point(right.col[i]+1,right.row[i]+1), Point(right.col[i]-1,right.row[i]-1),Scalar(0,0,255),2,8,0);
      }
      else {
        rectangle(frame_for_draw, Point(right.col[i]+1,right.row[i]+1), Point(right.col[i]-1,right.row[i]-1),Scalar(0,255,0),2,8,0);
      }
      if (mid.trust[i]){
        rectangle(frame_for_draw, Point(mid.col[i]+1,mid.row[i]+1), Point(mid.col[i]-1,mid.row[i]-1),Scalar(200,0,255),2,8,0);
      }
      else {
        rectangle(frame_for_draw, Point(mid.col[i]+1,mid.row[i]+1), Point(mid.col[i]-1,mid.row[i]-1),Scalar(200,255,0),2,8,0);
      }
    }
  }

  

  count=0;
  int final_index=0;
  for (int i=0; i<ram.LRS; i++){
  	if (mid.trust[i])	{
  		count++;
  		final_index+=mid.col[i];
  	}
  	if (count >= 5) break;
  }

  if (debug_process_image) imshow( "Warp", frame_for_draw );

  if (count >= 5) {
  	return ((384 - (final_index/5.0))/200.0);
  } else return -100;
  return -100;
 }

cv_bridge::CvImagePtr convert(const sensor_msgs::Image::ConstPtr& msg){
 	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return NULL;
    }

	return cv_ptr;
}

int change_lane_process(){
  float command_V_angular = present_command.float_2;
  float command_delta_d = present_command.float_1;

  int command_turn_direction = present_command.base_arg;
  int command_alpha = present_command.int_1;

  ram.change_lane_V_angular = command_V_angular*360/(2*3.141592);
  ram.change_lane_clk1 = (command_alpha*ram.Speed)/ram.change_lane_V_angular;
  ram.change_lane_alpha = (command_alpha*2*3.141592)/360;
  ram.change_lane_b = ram.Speed/ram.change_lane_V_angular;
  ram.change_lane_d1=((180*ram.change_lane_b)/(3.141592))*(1-cos(ram.change_lane_alpha));
  ram.change_lane_direction=command_turn_direction*2-3;
  ram.change_lane_clk2 = (command_delta_d - 2*ram.change_lane_d1)/sin(ram.change_lane_alpha);
  return (ram.change_lane_d1 > command_delta_d/2);
};

void contro_sig_recive(const demo_pakage::Num msg){
  switch (msg.header){
    case 1:{
      ram.FSM_state=1;
      break;
    }
    case 2:{
      printf("Recive Stop signal \n");
      ram.Speed=0;
      break;
    }
    case 3:{
      printf("Recive Move signal at speed %f \n",msg.float_1);
      ram.Speed=msg.float_1;
      break;
    }
    default:
    printf("Controller header unknow !");
  }
  
  present_command = msg;
}

void image_recive(const sensor_msgs::Image::ConstPtr& msg){

  //===== process time =====//
  ram.previous_time = ram.now_time;
  ram.now_time=ros::Time::now().toSec();
  if (ram.counter_FPS < 100){
    ram.counter_FPS++;
    ram.Now_FPS=(ram.Now_FPS*ram.counter_FPS + 1.0/(ram.now_time - ram.previous_time))/(ram.counter_FPS+1);
  } else {
    ram.Now_FPS = 0.98*ram.now_time + 0.02*(1.0/(ram.now_time - ram.previous_time));
  }

  //===== process speed =====//
  float tmp_speed = ram.Speed*ram.acceleration_ratio + (1-ram.acceleration_ratio)*ram.output_speed;
  if ((tmp_speed - ram.output_speed) > ram.acceleration_max){
    ram.output_speed += ram.acceleration_max;
  } else if ((tmp_speed - ram.output_speed) < -1*ram.acceleration_max){
    ram.output_speed -= ram.acceleration_max;
  } else ram.output_speed = tmp_speed;

	char c=(char)waitKey(3);
    if(c==27){
    	printf("\n ===> Sutdown Signal Recive <=== \n");
      printf("FPS: %3f \n",ram.Now_FPS);

			cv::destroyAllWindows();
			ros::shutdown();
    }

  switch(ram.FSM_state){
    case 0://==============================================
      {
        cv_bridge::CvImagePtr cv_image = convert(msg);
        float process_value=process(cv_image->image);
        if (process_value != -100 ) {
          ram.final_angular_velo = process_value*0.2 + ram.final_angular_velo*0.8;
          //printf("Lane detected \n");
        }
        //printf(" turn %4f => %4f \n",process_value, ram.final_angular_velo);
        data_msg.linear.x = ram.output_speed;
        data_msg.angular.z = ram.final_angular_velo;
        publish_data.publish(data_msg);
        break;
      }
    case 1://==============================================
        {
        
        if (ram.counter_state_1 == 3){
          data_msg.angular.z = ram.change_lane_direction*present_command.float_2*-1;
          ram.change_lane_remaning_S -= ram.output_speed*(ram.now_time - ram.previous_time);
          if (ram.change_lane_remaning_S <= 0) {
            ram.FSM_state = 0;
            ram.counter_state_1=0;
            printf("Change Lane complete! \n");
            break;
          }
        }
        
        if (ram.counter_state_1 == 2){
          data_msg.angular.z=0;
          ram.change_lane_remaning_S -= ram.output_speed*(ram.now_time - ram.previous_time);
          if (ram.change_lane_remaning_S <= 0) {
            ram.counter_state_1=3;
            printf("State 3 \n");
            ram.change_lane_remaning_S = ram.change_lane_clk1;
          }
        }

        if (ram.counter_state_1 == 1){
          data_msg.angular.z = ram.change_lane_direction*present_command.float_2;
          ram.change_lane_remaning_S -= ram.output_speed*(ram.now_time - ram.previous_time);
          if (ram.change_lane_remaning_S <= 0) {
            ram.counter_state_1=2;
            ram.change_lane_remaning_S = ram.change_lane_clk2;
            printf("State 2 \n");
          }
        }
        
        if (ram.counter_state_1 == 0){
          if (change_lane_process()){
            printf("Parameter is not sutable! exit state... \n");
            ram.FSM_state=0;
          }
          ram.counter_state_1=1;
          ram.change_lane_remaning_S = ram.change_lane_clk1;
          printf("State 1 \n");
        }

        
        data_msg.linear.x = ram.output_speed;
        publish_data.publish(data_msg);
      }
      break;
    default:  //==============================================
      printf("%d is Unknow state, do nothing !!!",ram.FSM_state);
      ram.FSM_state = 0;
  }

	//printf("frame:%3d process complete ! \n",ram.frame_count);
	ram.frame_count++;
  printf("output speed : %5f \n",ram.output_speed);
  printf("time : %5f \n ========== \n",ros::Time::now().toSec() - ram.now_time);
}

void Init_system(void){
  printf("System Initialize... \n");

  while (ros::Time::now().toSec() == 0);
  ram.now_time=ros::Time::now().toSec();
  printf("System Initialize Complete \n");
}

int main(int argc, char **argv){
	ros::init(argc,argv,"lane_detection");
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	ros::NodeHandle get_image;
	ros::Subscriber topic_sub = get_image.subscribe("/camera/rgb/image_raw",1000,image_recive);

  ros::NodeHandle contro_sig;
	ros::Subscriber another_topic_sub = contro_sig.subscribe("/controller_topic",1000,contro_sig_recive);

  ros::NodeHandle private_nh("~");

  ram.acceleration_max = private_nh.param<float>("acceleration_max",0.05);
  ram.acceleration_ratio = private_nh.param<float>("acceleration_ratio",0.4);


  Init_system();
	ros::spin();

	return 0;
}
