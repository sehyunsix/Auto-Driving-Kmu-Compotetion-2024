//
//  Preprocessing.hpp
//  Filter-Noise
//
//  Created by Yakup Görür on 31/07/2017.
//  Copyright © 2017 Yakup Görür. All rights reserved.
//

#ifndef Preprocessing_hpp
#define Preprocessing_hpp

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include <vector>

using namespace std;

cv::Mat Preprocessing(cv::Mat& src){
    bool debug = false;

    cv::Mat sourceHLS;
    cv::Mat channels[3];
    cv::Mat gray;
    cv::Mat source;


    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY); //Convert to RGB2Gray

//Gray Scale Preprocess

    //Debug Gray Scale Image
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/Original.jpg", src);
        cv::namedWindow( "Gray Scale Image", cv::WINDOW_NORMAL );
        cv::imshow("Gray Scale Image", gray);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

    cv::equalizeHist(gray, gray);   //Histogram equalization

    //Debug Histogram Equalization GrayScale
    if(debug){
        cv::namedWindow("Gray Scale Histogram Equalization", cv::WINDOW_NORMAL );
        cv::imshow("Gray Scale Histogram Equalization", gray);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

    cv::inRange(gray, cv::Scalar(125), cv::Scalar(255), gray); //Take threshold

    //Debug Threshold GrayScale
    if(debug){
        cv::namedWindow("Gray Scale Histequ-> Threshold", cv::WINDOW_NORMAL );
        cv::imshow("Gray Scale Histequ-> Threshold", gray);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }



    cv::cvtColor(src, sourceHLS, cv::COLOR_BGR2HLS);    //Convert Image RGB2HLS
    split(sourceHLS, channels); //Split color channels(H - L - S)

//S Channel PreProcess.

    //Debug S channel channels[2].
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/S_Channel.jpg", channels[2]);
        cv::namedWindow( "S-Channel", cv::WINDOW_NORMAL );
        cv::imshow("S-Channel", channels[2]);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

    cv::Mat S_channel(channels[2].size(), CV_8UC1);
    cv::inRange(channels[2], cv::Scalar(100), cv::Scalar(255), S_channel); //Take Threshold

    //Debug S channel Threshold channels[2].
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/Threshold_S_Channel.jpg", S_channel);
        cv::namedWindow( "Threshold- S Channel", cv::WINDOW_NORMAL );
        cv::imshow("Threshold- S Channel", S_channel);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

//L channel Process.

    //Debug L channel channels[1].
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/L_Channel.jpg", channels[1]);
        cv::namedWindow( "L-Channel", cv::WINDOW_NORMAL );
        cv::imshow("L-Channel", channels[1]);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }


    cv::Mat L_channel_Sx(channels[1].size(), CV_32FC1);  //Derivative in x direction of L Channel
    cv::Sobel(channels[1], L_channel_Sx, CV_32FC1, 1, 0); //Get derivative in x direction

    //Debug L Channel Derivative (Sobel)
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/Sobel_L_Channel.jpg", L_channel_Sx);
        cv::namedWindow( "Sobel- L Channel", cv::WINDOW_NORMAL );
        cv::imshow("Sobel- L Channel", L_channel_Sx);
        // cv::waitKey(0);

    }

    L_channel_Sx = cv::abs(L_channel_Sx);   //Take Absolute

    //Debug L Channel Sobel->Absolute
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/Sobel_L_Channel_absolute.jpg", L_channel_Sx);
        cv::namedWindow( "Sobel- L Channel-absolute", cv::WINDOW_NORMAL );
        cv::imshow("Sobel- L Channel-absolute", L_channel_Sx);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }


    double minVal, maxVal;
    minMaxLoc(L_channel_Sx, &minVal, &maxVal); //Find maximum value in image.

    cv::Mat L_channel(channels[1].size(), CV_8UC1); //Create Mat object to obtain L channel
    L_channel = L_channel_Sx.mul(255/maxVal); //Scale the L Channel between 0-255.

    //Debug L Channel Sobel->Absolute->Scale
    if(debug){
        cv::namedWindow("Sobel and Absolute and Scale- L Channel", cv::WINDOW_NORMAL );
        cv::imshow("Sobel and Absolute and Scale- L Channel", L_channel);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }


    cv::inRange(L_channel, cv::Scalar(20), cv::Scalar(100), L_channel); //Take Threshold

    //Debug L Channel Sobel->Absolute->Scale->Threshold
    if(debug){
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/Threshold_L_Channel.jpg", L_channel);
        cv::namedWindow( "Threshold- L Channel", cv::WINDOW_NORMAL );
        cv::imshow("Threshold- L Channel", L_channel);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }


//L || S
    cv::Mat combined_result(channels[1].size(), CV_8UC1); //Create combined_resuult objet

    bitwise_or(S_channel, L_channel, combined_result); //Bitwise or between L and S channels

    //Debug L Channel BitwiseOR S Channel
    if(debug){
        //            cv::namedWindow( "S-Channel-resultS", cv::WINDOW_NORMAL );cv::imshow("S-Channel-resultS", resultS);
        //            cv::namedWindow( "L-Channel-resultL", cv::WINDOW_NORMAL );cv::imshow("L-Channel-resultL", resultL);
        cv::imwrite("/Users/yakup/Test_output_Files/Debug/bitwiseOR_S_Channel_L_Channel.jpg", combined_result);
        cv::namedWindow( "bitwiseOR- S Channel || L Channel", cv::WINDOW_NORMAL );
        cv::imshow("bitwiseOR- S Channel || L Channel", combined_result);
        cv::imshow("gray", gray);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

//Gray & (L || S)
    bitwise_and(gray, combined_result, combined_result); //bitwise and GrayImage and (L || S)

    //Debug Gray Image BitwiseAND Combined Image
    if(debug){
        cv::namedWindow( "bitwiseAND- GRAY & (L||S)", cv::WINDOW_NORMAL );
        cv::imshow("bitwiseAND- GRAY & (L||S)", combined_result);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }

    return combined_result; //Result of the Preprocess Function
}

void edgeLines(cv::Mat img, cv::Mat &line_result, std::vector<cv::Vec4i> lines){
    int width = img.rows;
    
    for(size_t i = 0; i < lines.size(); i++){
        cv::Vec4i line = lines[i];
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];

        float slope;
        if(x2 == x1){ slope = 999.; }
        else{ slope = fabsf((y1 - y2) / float(x1 - x2)); }

        float line_row_theshold = tan(15*CV_PI/180);
        float line_high_threshold = tan(89.9*CV_PI/180);

        if (line_row_theshold < slope && slope < line_high_threshold){
            if(x1 < width / 2){
                // left side edge
                cv::line(line_result, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255,0,0), 2, cv::LINE_AA);
            } else {
                // right side edge
                cv::line(line_result, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0,255,0), 2, cv::LINE_AA);
            }
        }
    }
}

cv::Mat New_Preprocessing(cv::Mat& src){
    bool debug = false;

    cv::Mat sourceHLS;
    cv::Mat channels[3];
    cv::Mat gray;
    cv::Mat canny;
  
    double rho = 1;
    double theta = CV_PI/180;
    int threshold = 10;
    double minLineLength = 0;
    double maxLineGap = 0;

    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY); //Convert to RGB2Gray
    
    std::vector<cv::Vec4i> lines;
    
    cv::Canny(gray,canny,100,200,3,false);
    cv::imshow("Canny", canny);




    // cv::imshow("ORIGINAL", src);
    // cv::imshow("RESULT", result);

   

    return canny; //Result of the Preprocess Function
}


#endif /* Preprocessing_hpp */
