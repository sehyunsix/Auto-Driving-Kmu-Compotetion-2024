//
//  main.cpp
//  Advance_Lane_Detection
//
//  Created by Yakup Gorur on 2/10/17.
//  Copyright © 2017 Yakup Gorur. All rights reserved.
//
#include <iostream>
#include <experimental/filesystem>

#include <time.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Preprocessing.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dirent.h>
#include <string>
#include<algorithm>



using namespace cv;
using namespace std;

std::vector<std::string> find_jpg_images(const std::string& path) {
    std::vector<std::string> jpg_files;
    DIR* dir;
    struct dirent* entry;

    dir = opendir(path.c_str());
    if (dir == nullptr) {
        throw std::runtime_error("The specified path does not exist: " + path);
    }

    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        std::string extension = filename.substr(filename.find_last_of(".") + 1);

        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == "jpg" || extension == "jpeg") {
            jpg_files.push_back(path + "/" + filename);
        }
    }

    closedir(dir);
    return jpg_files;
}


bool debug = false;
bool boolclk_preprocess = true;
bool boolclk_video = true;

int xsize = 640, ysize = 480; // Frame width and height
int poly_degree=3;
int global = 1; //??
Mat M, Minv, frame, result;

    Point2f src[] = {Point2f(0, ysize*4.5/6.0), Point2f(xsize*1.5/6.0, ysize*3.6/6.0), Point2f(xsize*4.8/6.0, ysize*3.3/6.0), Point2f(xsize, ysize*3.9/6.0)};
    Point2f dst[] = {Point2f(xsize*0.20, ysize), Point2f(xsize*0.20, 0), Point2f(xsize*0.80, 0), Point2f(xsize*0.80, ysize)};      




void processVideo(string videoFilename);
void LineDetect(cv::Mat binary_warped, float* left_fit, float* right_fit);
void DrawLines(cv::Mat binary_warped, float* left_fit, float* right_fit);
void DrawLane(cv::Mat frame, cv::Mat &result, float* left_fit, float* right_fit);
void SlidingWindow(cv::Mat binary_warped);
void FollowLine(cv::Mat binary_warped);
void imageCallBack(const sensor_msgs::ImageConstPtr& msgs);
void laneDrive();

//Todo
void pidcontrol();
void make_drvie_angle(); 



class FloatHistogram {
private:
    std::vector<float> data;
    std::vector<int> histogram;
    std::vector<float> bin_edges;
    int num_bins;
    float min_value, max_value;

public:
    FloatHistogram(const std::vector<float>& input_data, int bins) : data(input_data), num_bins(bins) {
        min_value = *std::min_element(data.begin(), data.end());
        max_value = *std::max_element(data.begin(), data.end());
        
        computeHistogram();
    }

    void computeHistogram() {
        histogram.resize(num_bins, 0);
        bin_edges.resize(num_bins + 1);

        float bin_width = (max_value - min_value) / num_bins;
        if(bin_width<=0.0001){ class FloatHistogram {
private:
    std::vector<float> data;
    std::vector<int> histogram;
    std::vector<float> bin_edges;
    int num_bins;
    float min_value, max_value;

public:
    FloatHistogram(const std::vector<float>& input_data, int bins) : data(input_data), num_bins(bins) {
        min_value = *std::min_element(data.begin(), data.end());
        max_value = *std::max_element(data.begin(), data.end());
        
        computeHistogram();
    }

    void computeHistogram() {
        histogram.resize(num_bins, 0);
        bin_edges.resize(num_bins + 1);

        float bin_width = (max_value - min_value) / num_bins;
        if(bin_width<=0.0001){ 

            bin_edges[0]=data[0];
            bin_edges[1]=data[1];
            histogram[0]++;
            return;
        }

        for (int i = 0; i <= num_bins; ++i) {
            bin_edges[i] = min_value + i * bin_width;
        }

        for (float value : data) {
            
            int bin = std::min(static_cast<int>((value - min_value) / bin_width), num_bins - 1);
            // fprintf(stderr,"debugbin: %d , value: %.2f\n min vlaue: %.2f bin width : %.2f",bin,value,min_value,bin_width);
            histogram[bin]++;
        }
    }

    void removeSmallCountAnomalies(int threshold) {
        std::vector<float> filtered_data;
        
        for (float value : data) {
            int bin = std::min(static_cast<int>((value - min_value) / (max_value - min_value) * num_bins), num_bins - 1);
            if (histogram[bin] > threshold) {
                filtered_data.push_back(value);
            }
        }
        
        data = filtered_data;
        computeHistogram();
    }

    void printHistogram() {
        for (int i = 0; i < num_bins; ++i) {
            std::cout << "[" << bin_edges[i] << ", " << bin_edges[i+1] << "): " << histogram[i] << std::endl;
        }
    }

    const std::vector<float>& getFilteredData() const {
        return data;
    }

    double getMaxHistogramData(){
        fprintf(stderr,"found max value error\n");
        int maxindex=std::max_element(histogram.begin(),histogram.end())-histogram.begin();
        double data=(bin_edges[maxindex]+bin_edges[maxindex+1])/2;
        return data;
    }

};

            bin_edges[0]=data[0];
            bin_edges[1]=data[1];
            histogram[0]++;
            return;
        }

        for (int i = 0; i <= num_bins; ++i) {
            bin_edges[i] = min_value + i * bin_width;
        }

        for (float value : data) {
            
            int bin = std::min(static_cast<int>((value - min_value) / bin_width), num_bins - 1);
            // fprintf(stderr,"debugbin: %d , value: %.2f\n min vlaue: %.2f bin width : %.2f",bin,value,min_value,bin_width);
            histogram[bin]++;
        }
    }

    void removeSmallCountAnomalies(int threshold) {
        std::vector<float> filtered_data;
        
        for (float value : data) {
            int bin = std::min(static_cast<int>((value - min_value) / (max_value - min_value) * num_bins), num_bins - 1);
            if (histogram[bin] > threshold) {
                filtered_data.push_back(value);
            }
        }
        
        data = filtered_data;
        computeHistogram();
    }

    void printHistogram() {
        for (int i = 0; i < num_bins; ++i) {
            std::cout << "[" << bin_edges[i] << ", " << bin_edges[i+1] << "): " << histogram[i] << std::endl;
        }
    }

    const std::vector<float>& getFilteredData() const {
        return data;
    }

    double getMaxHistogramData(){
        fprintf(stderr,"found max value error\n");
        int maxindex=std::max_element(histogram.begin(),histogram.end())-histogram.begin();
        double data=(bin_edges[maxindex]+bin_edges[maxindex+1])/2;
        return data;
    }

};

int main(int argc, char * argv[]) {
    string filename;
    string path_image = "/home/xytron/xycar_ws/data/";

  
    cin >> filename;
    Mat mask = Mat::zeros(ysize ,xsize, CV_8UC3);

    std::vector<std::string> jpg_images;
    try {
        jpg_images = find_jpg_images(path_image);

        // 찾은 JPG 파일 출력   
        for (const auto& img : jpg_images) {
            std::cout << img << std::endl;
        }
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
    }

    for(auto &it:jpg_images){
    // Calculate transformation matrices
    string tmp;
    M = getPerspectiveTransform(src, dst);
    Minv = getPerspectiveTransform(dst, src);
    fprintf(stdout, "start_now\n");
    frame =cv::imread(it);
   
    fprintf(stdout, "start_now\n");
     laneDrive();
    
    }

    // laneDrive();
    
    // cin >> tmp;
    // if (tmp=="s"){break;}

    // }

    // cv::destroyWindow("view");

    return 0;
}




void laneDrive(){



   
    Mat mask = Mat::zeros(ysize ,xsize, CV_8UC3);

    // 다각형으로 ROI 채우기
    Point rook_points[1][4];
    rook_points[0][0] = src[0];
    rook_points[0][1] = src[1];
    rook_points[0][2] = src[2];
    rook_points[0][3] = src[3];

    const Point* ppt[1] = { rook_points[0] };
    int npt[] = { 4 };

    fillPoly(mask, ppt, npt, 1, Scalar(255, 255, 255), 8);

    // 원본 이미지와 마스크를 사용하여 ROI를 추출
    Mat maskedImage;
    bitwise_and(frame, mask, maskedImage);

    // 결과 이미지 보여주기
    // imshow("Source Image", frame);
    // imshow("Mask", mask);
    imshow("Masked Image", maskedImage);

    // Calculate transformation matrices
    M = getPerspectiveTransform(src, dst);
    Minv = getPerspectiveTransform(dst, src);
    // fprintf(stdout, "start_now\n");
    Mat binary(xsize, ysize, CV_8UC1);
    binary = Preprocessing(frame);
  

    // fprintf(stdout, "get perspetctive of image\n");
    // Get perspective of image
    Mat binary_warped(xsize, ysize, CV_8UC1);
    warpPerspective(binary, binary_warped, M, Size(xsize, ysize), INTER_LINEAR);
    imshow("binary_warped", binary_warped);



    binary = New_Preprocessing(frame);


    warpPerspective(binary, binary_warped, M, cv::Size(xsize, ysize), cv::INTER_LINEAR);
    cv::imshow("warpCanny", binary_warped);
    
    double rho = 1;
    double theta = CV_PI/180;
    int threshold = 200;
    double minLineLength = 70;
    double maxLineGap = 10;

    std::vector<cv::Vec4i> lines;
    Mat cdst;
    cv::HoughLinesP(binary_warped, lines, rho, theta, threshold, minLineLength, maxLineGap);
    cvtColor(binary_warped, cdst, COLOR_GRAY2BGR);
    fprintf(stderr,"lines numberr %d",lines.size() );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
    }

    // cv::Mat line_result = cv::Mat::zeros(binary_warped.size(), CV_8UC3);
    // edgeLines(binary_warped, line_result, lines);
   

    cv::imshow("Houglines", cdst);


    double sum_angle = 0.0;
    int count=0;
    vector<float> theta_list;

    for (size_t i = 0; i < lines.size(); i++) {
        
        if((lines[i][2] -lines[i][0])==0){
            continue;
        }
        float theta;
        // if(lines[i][3]>lines[i][1]) {
         
        //  theta =-(lines[i][3] -lines[i][1])/(lines[i][2] -lines[i][0]);
        // }
        // else{
        
         theta = -(lines[i][3] -lines[i][1])/(lines[i][2] -lines[i][0]);
        // }

        double angle = atan(theta);
        
        if(angle <0){angle +=CV_PI;}
        theta_list.push_back(angle);
        fprintf(stderr,"amgle : %.2f\n",angle);
        fprintf(stderr, "cosine :%.2f\n",cos(angle) );
        // Convert theta to degrees
        }

     if (theta_list.size() ==0 ){
        return;
     }
    fprintf(stderr,"make historgram numberr \n" );
    FloatHistogram histogram(theta_list, 20);

    std::cout << "Original Histogram:" << std::endl;
    histogram.printHistogram();

    // histogram.removeSmallCountAnomalies(threshold);

    std::cout << "\nHistogram after removing anomalies:" << std::endl;
    histogram.printHistogram();

    std::cout << "\nFiltered data:" << std::endl;
    // for (float value : histogram.getFilteredData()) {
    //     std::cout << value << " ";
    // }

    std::cout << std::endl;
    

    // Calculate the average angle
    double avg_angle;
    if (theta_list.size() > 0) {
        avg_angle = histogram.getMaxHistogramData();
        std::cout << "Average line angle: " << avg_angle << " degrees" << std::endl;
    } else {
        std::cout << "No lines detected" << std::endl;
        return;
    }

    // Plot the average angle line with arrow
    int bottom_margin = 50;
    int line_length = 100;
   
    cv::Point center(frame.cols / 2, frame.rows - bottom_margin);
    cv::Point end_point(center.x + line_length * cos(avg_angle),
                        center.y - line_length *sin(avg_angle));

    cv::line(frame, center, end_point, cv::Scalar(0, 255, 0), 2);
    cv::arrowedLine(frame, center, end_point, cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);

    // Add text to show the angle
    std::string angle_text = "Avg Angle: " + std::to_string(int((avg_angle/(CV_PI))*180)) + "°";
    cv::putText(frame, angle_text, cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    // Display the result
    cv::imshow("Result", frame);

    // fprintf(stdout, "get perspetctive of image\n");
    // Get perspective of image
    // Mat result;
    // warpPerspective(binary, binary_warped, M, Size(xsize, ysize), INTER_LINEAR);
    // // cv::Canny(binary_warped,result,100,50,3,false);
    // imshow("new binary_warped", binary_warped);
    waitKey(0);
    cv::destroyAllWindows();

  

}
