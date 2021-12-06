#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"

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


ros::Publisher publish_data;
geometry_msgs::Twist data_msg;

#define pixel(f,i,c) (int)(*f.ptr(i,c))

#define Speed 0.1

#define CSR 15
#define RLS 25   //Range of Lane Search
#define LRS 40   //Long of Lane Search


int frame_count=0;
float final_velo=0;

int FSM_state=0;

int counter_state_1=0;

demo_pakage::Num present_command;

class Lane{
public:
  int col[LRS]={0}; //col index
  int row[LRS]={0}; //row index
  bool trust[LRS]={0};

  void checkCSR(void){
    for(int i=1; i<LRS-1;i++){
      float tmp=(col[i-1]+col[i+1])/2.0-col[i];
      //printf("check %d: %d and %d => %d \n",i,col[i],trust[i],((tmp*tmp < CSR) & trust[i]));
      trust[i]=((tmp*tmp < CSR) & trust[i]);
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
  auto begin = std::chrono::high_resolution_clock::now();
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

  Mat cut_for_sum=warp(Range(448,512),Range(128,640));
  
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
    if (tmp > 5000) break;
    center++;
  }
  int right_start=center+128;

  center=255;
  while (center >0){
    int tmp=0;
    for (int i=0;i<64;i++){
      tmp=tmp+(int)(*cut_for_sum.ptr(i,center));
    }
    if (tmp > 5000) break;
    center--;
  }

  int left_start=center+128;

  Point p1(left_start,0), p2(left_start,512);
  line(frame_for_draw, p1, p2, Scalar(255,0,0), 2, LINE_4);
  p1=Point(right_start,0);
  p2=Point(right_start,512);
  line(frame_for_draw, p1, p2, Scalar(255,0,0), 2, LINE_4);

  //================================ detect started ===============================

  Lane left, right, mid, trust;

  int sample_jump=5;
  int count=0;
  int check_row=500;
  // Left check
  while (check_row > 500-LRS*sample_jump){
    for (int i=left_start+RLS; i>left_start-RLS; i--){
      if (pixel(frame,check_row,i) != 0){
        left.col[count]=i;
        left.trust[count]=1;
        //rectangle(frame_for_draw, Point(i+1, check_row+1), Point(i-1,check_row-1),Scalar(0,0,255),2,8,0);
        left_start=i;
        break;
      }
    }
    left.row[count]=check_row;
    count++;
    check_row-=sample_jump;
  }

  count=0;
  check_row=500;

  while (check_row > 500-LRS*sample_jump){
    for (int i=right_start-RLS; i<right_start+RLS; i++){
      if (pixel(frame,check_row,i) != 0){
        right.col[count]=i;
        right.trust[count]=1;
        //rectangle(frame_for_draw, Point(i+1, check_row+1), Point(i-1,check_row-1),Scalar(0,100,255),2,8,0);
        right_start=i;
        break;
      }
    }

    right.row[count]=check_row;
    count++;
    check_row-=sample_jump;
  }
  
  //================================ CSR check ===============================
  left.checkCSR();
  right.checkCSR();

  for (int i=0; i<LRS; i++){
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
    mid.row[i]=left.row[i];
    mid.col[i]=(left.col[i] + right.col[i])/2;
    mid.trust[i]=(left.trust[i] & right.trust[i]);

    if (mid.trust[i]){
      rectangle(frame_for_draw, Point(mid.col[i]+1,mid.row[i]+1), Point(mid.col[i]-1,mid.row[i]-1),Scalar(200,0,255),2,8,0);
    }
    else {
      rectangle(frame_for_draw, Point(mid.col[i]+1,mid.row[i]+1), Point(mid.col[i]-1,mid.row[i]-1),Scalar(200,255,0),2,8,0);
    }
  }

  count=0;
  int final_index=0;
  for (int i=0; i<LRS; i++){
  	if (mid.trust[i])	{
  		count++;
  		final_index+=mid.col[i];
  	}
  	if (count >= 5) break;
  }

  imshow( "Warp", frame_for_draw );

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  //printf("\n =============================== \n Time measured: %.5f seconds. \n", elapsed.count() * 1e-9);

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

void contro_sig_recive(const demo_pakage::Num msg){
  switch (msg.header){
    case 1:{
      FSM_state=1;
      break;
    }
    default:
    printf("Controller header unknow !");
  }
  
  present_command = msg;
}

void ReturnFunc(const sensor_msgs::Image::ConstPtr& msg){

	cv_bridge::CvImagePtr cv_image = convert(msg);
	
	string windown_name = "Image";

	imshow(windown_name, cv_image->image);

	char c=(char)waitKey(3);
    if(c==27){
    	printf("\n ===> Sutdown ! <=== \n");
			cv::destroyAllWindows();
			ros::shutdown();
    }

  switch(FSM_state){
    case 0://==============================================
      {
        float process_value=process(cv_image->image);
        if (process_value != -100 ) {
          final_velo = process_value*0.2 + final_velo*0.8;
          //printf("Lane detected \n");
        } else {
          printf(" => Lane undetected !!! turn %4f \n",data_msg.angular.z);
        }
        //printf(" turn %4f => %4f \n",process_value, final_velo);
        data_msg.linear.x = Speed;
        data_msg.angular.z = final_velo;
        publish_data.publish(data_msg);
        break;
      }
    case 1://==============================================
        {
        printf("STATE 1 started %d time \n",counter_state_1);

        counter_state_1++;

        float V_angular = present_command.float_1*360/(2*3.141592);
        int clk1 = 20*present_command.int_1/V_angular;
        float alpha = (present_command.int_1*2*3.141592)/360;
        float b = Speed/V_angular;
        float d1=((180*b)/(3.141592))*(1-cos(alpha));
        int direction=present_command.base_arg*2-3;
        int clk2 = (present_command.float_2 - (360*b/3.141592)*(1-cos(alpha)))*20/(Speed*sin(alpha));

        clk1/=2;
        clk2/=2;

        if (d1 > present_command.float_2/2) {
          printf("Parameter out of range! %f and %f \n",d1,present_command.float_2);
          printf("I'll not move !");
          FSM_state=0;
          break;
        }
                
        data_msg.linear.x = Speed;

        printf("CLK1: %d  and CLK2:%d !\n",clk1,clk2);

        if (counter_state_1 < clk1){
          data_msg.angular.z=-1*present_command.float_1*direction;
          publish_data.publish(data_msg);
        } else if (counter_state_1 < clk1 + clk2) {
          data_msg.angular.z=0;
          publish_data.publish(data_msg);
        }else if (counter_state_1 < clk1*2 + clk2){
          data_msg.angular.z=present_command.float_1*direction;
          publish_data.publish(data_msg);
        } else {
          printf("Complete state 1, exit state \n");
          FSM_state=0;
          counter_state_1=-1;
        }

      }
      break;
    default:  //==============================================
      printf("%d is Unknow state, do nothing !!!",FSM_state);
  }

	//printf("frame:%3d process complete ! \n",frame_count);
	frame_count++;

}

int main(int argc, char **argv){
	ros::init(argc,argv,"sensor_read");
	
	ros::NodeHandle nh;
	publish_data = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	ros::NodeHandle get_image;
	ros::Subscriber topic_sub = get_image.subscribe("/camera/rgb/image_raw",1000,ReturnFunc);

  ros::NodeHandle contro_sig;
	ros::Subscriber another_topic_sub = contro_sig.subscribe("/controller_topic",1000,contro_sig_recive);

	ros::spin();

	return 0;
}