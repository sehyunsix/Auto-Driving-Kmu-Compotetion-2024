//
//  main.cpp
//  Advance_Lane_Detection
//
//  Created by Yakup Gorur on 2/10/17.
//  Copyright © 2017 Yakup Gorur. All rights reserved.
//
#include <iostream>
#include <time.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Preprocessing.hpp"
#include "Lane.hpp"
#include "lidar.hpp"
#include "Fitting_Second_Order.hpp"
#include "TimeAnalyze.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "xycar_msgs/xycar_motor.h"
#include <string>


using namespace cv;
using namespace std;


#define  xsize 640
#define  ysize 480

bool debug = false;
bool boolclk_preprocess = true;
bool boolclk_video = true;

// Frame width and height
int poly_degree=2;
int global = 1; //??
int angle =0;
float speed =5;
float lidar_point[LIDAR_SIZE];

Mat M, Minv, frame, result;
Point2f src[] = {Point2f(0, ysize*4.5/6.0), Point2f(xsize*1.5/6.0, ysize*3.6/6.0), Point2f(xsize*4.8/6.0, ysize*3.3/6.0), Point2f(xsize, ysize*3.9/6.0)};
Point2f dst[] = {Point2f(xsize*0.20, ysize), Point2f(xsize*0.20, 0), Point2f(xsize*0.80, 0), Point2f(xsize*0.80, ysize)};      


void DrawLines(cv::Mat binary_warped, float* left_fit, float* right_fit);
void DrawLane(cv::Mat frame, cv::Mat &result, float* left_fit, float* right_fit);
void imageCallBack(const sensor_msgs::ImageConstPtr& msgs);
void lidarCallBack(const sensor_msgs::LaserScan::ConstPtr& msgs);
void laneDrive();
void lane_driving( Mat frame,  ros::Publisher& pub);
void lidar_driving( float* lidar_point, ros::Publisher& pub);
void drive(int angle, int speed, ros::Publisher& pub);

int main(int argc, char * argv[]) {

    // Source and Destination points for perspective transformation coefficients

    Mat mask = Mat::zeros(ysize ,xsize, CV_8UC3);
    // Calculate transformation matrices
    M = getPerspectiveTransform(src, dst);
    Minv = getPerspectiveTransform(dst, src);
    fprintf(stdout, "start_now\n");



    ros::init(argc,argv,"image_subscriber");
    ros::NodeHandle nh;
    ros::Rate  r(30);
    cv::namedWindow("view");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub =it.subscribe("/usb_cam/image_raw/",1 ,imageCallBack);
    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidarCallBack);
    
    ros::Publisher pub= nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor",1);

   
    Mat binary(xsize, ysize, CV_8UC1);
    Mat binary_warped(xsize, ysize, CV_8UC1);
    
    while(ros::ok()){
        if (!frame.empty()){
            lane_driving(frame, pub);
            // lidar_driving(lidar_point,pub);
        }
        ros::spinOnce();

        // r.sleep();

    }
    cv::destroyWindow("view");
    // processVideo(u_path_video); //ProcessVideo


    return 0;
    
}

void lidarCallBack(const sensor_msgs::LaserScan::ConstPtr& msgs){
    fprintf(stderr,"call back\n");
    for(int i=0; i<LIDAR_SIZE; i++){
        lidar_point[i] = msgs->ranges[i];
        fprintf(stderr," %f", msgs->ranges[i]);            

    }
}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg){
  try{
    
    // fprintf(stdout, "get perspetctive of image\n");
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("view",frame);
    cv::waitKey(1);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void drive(int angle, int speed, ros::Publisher& pub){
    xycar_msgs::xycar_motor msg;
    msg.angle = angle;
    msg.speed = speed;
    pub.publish(msg);
}

void lane_driving( Mat frame, ros::Publisher& pub){
    
    Mat binary(xsize, ysize, CV_8UC1);
    Mat binary_warped(xsize, ysize, CV_8UC1);

    fprintf(stderr,"roscpp cycle start");
        
    DrawMask(src, 4, ysize, xsize, frame);
    
    binary = New_Preprocessing(frame);
    
    warpPerspective(binary, binary_warped, M, Size(xsize, ysize), INTER_LINEAR);
    
    imshow("binary_warped", binary_warped);

    int new_angle =houghline_detect(binary_warped);

    DrawArrow(new_angle, frame);

    // cv::waitKey(0);

    drive(new_angle, 0, pub);


}


void lidar_driving(float* lidar_point, ros::Publisher& pub){
    
    vector<int> boundary_point_list = find_boundary_point(lidar_point);
    int new_angle = find_angle(boundary_point_list, lidar_point);
    // fprintf(stderr,"angle : %d\n",new_angle);


    xycar_msgs::xycar_motor msg;
    msg.angle = new_angle;
    msg.speed = 0;
    pub.publish(msg);
    DrawArrow(new_angle, frame);

}


