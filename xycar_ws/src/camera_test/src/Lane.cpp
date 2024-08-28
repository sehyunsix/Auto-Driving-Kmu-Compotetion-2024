//
//  Lane.cpp
//  Lane_Detection(local)
//
//  Created by Yakup Görür on 08/08/2017.
//  Copyright © 2017. All rights reserved.
//

#include "Lane.hpp"
#include<vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

Lane::Lane(int row, int col){
    buffer=10;
    buffer_coefficient=-1;

    //for CalculateLineDistance()
    buffer_lane_calculations=-1;
    buffer_lane_x_l=-1;
    buffer_lane_x_r=-1;

    detected_right = false;
    detected_left = false;
    donotdraw=true;
    y_size=row;     //rows-height of frame
    x_size=col;     //cols-width of frame
    image_center_x = x_size/2; //Generally

    coefficient_left_recent.resize(buffer);
    coefficient_right_recent.resize(buffer);
    distance_center_left_recent.resize(buffer);
    distance_right_center_recent.resize(buffer);
    right_lane_x_indis_recent.resize(buffer);
    left_lane_x_indis_recent.resize(buffer);
    distance_right_left_recent.resize(buffer);

}

int Lane::GetXValue(float* fit, int col){
    return int( fit[0]*pow(col,2) + fit[1]*col + fit[2]);
}

bool Lane::CheckIsItLane(float *left, float *right){

    int XINTERVAL = 60; //floating of line rigt-left (Line sağa sola kayması)

    int l_lane_x[3], r_lane_x[3];
    l_lane_x[0] = GetXValue(left,  y_size);
    r_lane_x[0] = GetXValue(right, y_size);

    l_lane_x[1] = GetXValue(left,  (y_size*80) /100);
    r_lane_x[1] = GetXValue(right, (y_size*80) /100);

    l_lane_x[2] = GetXValue(left,  (y_size*50) /100);
    r_lane_x[2] = GetXValue(right, (y_size*50) /100);



    //right-left difference
    int minb, maxb;
    int diff_r_l[3];
    minb=
    maxb=
    diff_r_l[0] = ( r_lane_x[0] - l_lane_x[0] );
    diff_r_l[1] = ( r_lane_x[1] - l_lane_x[1] );
    diff_r_l[2] = ( r_lane_x[2] - l_lane_x[2] );

    for(int i=1; i<3; i++){
        if(diff_r_l[i]>maxb){
            maxb = diff_r_l[i];
        }
        if(diff_r_l[i]<minb){
            minb = diff_r_l[i];
        }
    }
    int diff_l[3], diff_r[3];
    int minl, maxl;


    minl=
    maxl=
    diff_l[0] = abs(l_lane_x[0] - left_lane_x_indis_recent[buffer_lane_x_l][0]);
    diff_l[1] = abs(l_lane_x[1] - left_lane_x_indis_recent[buffer_lane_x_l][1]);
    diff_l[2] = abs(l_lane_x[2] - left_lane_x_indis_recent[buffer_lane_x_l][2]);

    int minr, maxr;
    minr=
    maxr=
    diff_r[0] = abs(r_lane_x[0]- right_lane_x_indis_recent[buffer_lane_x_r][0]);
    diff_r[1] = abs(r_lane_x[1]- right_lane_x_indis_recent[buffer_lane_x_r][1]);
    diff_r[2] = abs(r_lane_x[2]- right_lane_x_indis_recent[buffer_lane_x_r][2]);

    //left ne kadar oynamış? right nekadar oynamış?
    for(int i=1; i<3; i++){
        if(diff_l[i]>maxl){
            maxl = diff_l[i];
        }
        if(diff_l[i]<minl){
            minl = diff_l[i];
        }


        if(diff_r[i]>maxr){
            maxr = diff_r[i];
        }
        if(diff_r[i]<minr){
            minr = diff_r[i];
        }

    }


/* TODO Is it Lane
     
    //Conditions:
    //Gürültüden bozulmayı azaltır bu condition.
    //Right X_index aynı değermi?
    if(maxr < XINTERVAL){ //AYNI

        //Left X_index aynı değermi?
        if (maxl < XINTERVAL) { // Aynı [1,1]
            //Recente ekle ikisinide. //recente ekleme burada henüz olmuyor. Ama addcoefficient'da oluyor
        } else { //Aynı Değil [1,0]
            //Left'i Recent'ten çek. Right'ı ekle. //recente ekleme burada henüz olmuyor. Ama addcoefficient'da oluyor
            left= coefficient_left_recent[buffer_coefficient];
        }
    }
    else{   //AYNI DEĞİL
        //Left X_index aynı değermi?
        if (maxl < XINTERVAL) { // Aynı [0,1]
            //Right'i Recent'ten çek. Left'i ekle.
            right=coefficient_right_recent[buffer_coefficient];
        } else { //Aynı Değil [0,0]
            //Birşey yapma next condition yapar zaten.
        }

    }
    
*/



    // // //Lane genişliği uygun mu?
    // if ( ( (maxb-minb) > 350) || (minb < 200) || (maxb > 975) ) { //değil
    //     //Eski iki değeri al yada başarız
    //      return false;
    // } else {
    //     //süper. Şerit Değişikliğini kontrol ettin mi? Bir sonraki condition.//TODO
    //     return true;
    // }
    return true;

}




void Lane::AddCoefficent(float *left, float *right){

    // float tmp =0;
    // tmp =left[0];
    // left[0] =left[2];
    // left[2] =tmp;
    // tmp =right[0];
    // right[0] =right[2];
    // right[2] =tmp;
    //for first frame
    if(buffer_lane_x_r==-1 || buffer_lane_x_l==-1){
        detected_left = detected_right = true;
        CalculateLineDistances(left, right);
    }

    if( CheckIsItLane(left, right) ) {
        //Adding right & left lane coefficients
        coefficient_left_current[0]  = left[0];
        coefficient_left_current[1]  = left[1];
        coefficient_left_current[2]  = left[2];

        coefficient_right_current[0] = right[0];
        coefficient_right_current[1] = right[1];
        coefficient_right_current[2] = right[2];


        //ADDING RECENT COEFFECIENT (little bit complicated)
        //Note:dynamic memory to keep the recent coefficents in vector
        coefficient_left_recent_Mem_allocate  = new float [3];
        coefficient_right_recent_Mem_allocate = new float [3];

        coefficient_left_recent_Mem_allocate[0] = left[0];
        coefficient_left_recent_Mem_allocate[1] = left[1];
        coefficient_left_recent_Mem_allocate[2] = left[2];

        coefficient_right_recent_Mem_allocate[0] = right[0];
        coefficient_right_recent_Mem_allocate[1] = right[1];
        coefficient_right_recent_Mem_allocate[2] = right[2];


        //index increase
        ++buffer_coefficient;

        buffer_coefficient %= buffer;


        //Average calculation
        //( (average*10) - SonDeğer + YeniDeğer ) /10
        if(coefficient_right_recent [buffer_coefficient] != NULL){
            //Right yeni değer
            float *Dright= coefficient_right_recent [buffer_coefficient];
            //Left yeni değer
            float *Dleft= coefficient_left_recent [buffer_coefficient];


            coefficient_left_average[0] =  ( (coefficient_left_average[0]*10) + coefficient_left_recent_Mem_allocate[0] - Dleft[0]  ) / (10);

            coefficient_left_average[1] = ( (coefficient_left_average[1]*10) + coefficient_left_recent_Mem_allocate[1] - Dleft[1]  ) / (10);
            coefficient_left_average[2] = ( (coefficient_left_average[2]*10) + coefficient_left_recent_Mem_allocate[2] - Dleft[2]  ) / (10);
            coefficient_right_average[0] = ( (coefficient_right_average[0]*10) + coefficient_right_recent_Mem_allocate[0] - Dright[0]  ) / (10);
            coefficient_right_average[1] = ( (coefficient_right_average[1]*10) + coefficient_right_recent_Mem_allocate[1] - Dright[1]  ) / (10);
            ;
            coefficient_right_average[2] = ( (coefficient_right_average[2]*10) + coefficient_right_recent_Mem_allocate[2] - Dright[2]  ) / (10);
            ;

        }
        //just for first "buffer" time (Daha ilk circle dolmadan)
        // ((ilkdeğer * buffer) + yeni değer ) / (buffer+1)
        else{
            coefficient_left_average[0]  = ( (coefficient_left_average[0]*buffer_coefficient ) + coefficient_left_recent_Mem_allocate[0]  ) / (buffer_coefficient +1);
            coefficient_left_average[1]  = ( (coefficient_left_average[1]*buffer_coefficient ) + coefficient_left_recent_Mem_allocate[1]  ) / (buffer_coefficient +1);
            coefficient_left_average[2]  = ( (coefficient_left_average[2]*buffer_coefficient ) + coefficient_left_recent_Mem_allocate[2]  ) / (buffer_coefficient +1);
            coefficient_right_average[0] = ( (coefficient_right_average[0]*buffer_coefficient) + coefficient_right_recent_Mem_allocate[0] ) / (buffer_coefficient +1);
            coefficient_right_average[1] = ( (coefficient_right_average[1]*buffer_coefficient) + coefficient_right_recent_Mem_allocate[1] ) / (buffer_coefficient +1);
            coefficient_right_average[2] = ( (coefficient_right_average[2]*buffer_coefficient) + coefficient_right_recent_Mem_allocate[2] ) / (buffer_coefficient +1);
        }

        //Note:First deleting the memory then adding on the current index
        //Adding Recent right & left lane coefficients
        delete [] coefficient_left_recent [ (buffer_coefficient) ] ;
        delete [] coefficient_right_recent [ (buffer_coefficient) ] ;
        coefficient_left_recent [ (buffer_coefficient) ] = coefficient_left_recent_Mem_allocate;
        coefficient_right_recent [ (buffer_coefficient) ] = coefficient_right_recent_Mem_allocate;

        detected_left=true;
        detected_right=true;

//        //hiç girmiyor.
//        if(!(CheckIsItLane(coefficient_left_average, coefficient_right_average) )){
//            coefficient_right_average[2]=0;
//            coefficient_right_average[1]=0;
//            coefficient_right_average[0]=0;
//            coefficient_left_average[2]=0;
//            coefficient_left_average[1]=0;
//            coefficient_left_average[0]=0;
//        }
        //Calculate Line Distance now
        CalculateLineDistances(coefficient_left_current,coefficient_right_current);
    }
    else{
        coefficient_left_current[0] = coefficient_left_average[0];
        coefficient_left_current[1] = coefficient_left_average[1];
        coefficient_left_current[2] = coefficient_left_average[2];

        coefficient_right_current[0] = coefficient_right_average[0];
        coefficient_right_current[1] = coefficient_right_average[1];
        coefficient_right_current[2] = coefficient_right_average[2];

        //Hiç girmiyor.
        if(!(CheckIsItLane(coefficient_left_average, coefficient_right_average) )){
            coefficient_left_current[0]=0;
            coefficient_left_current[1]=0;
            coefficient_left_current[2]=0;
            coefficient_right_current[2]=0;
            coefficient_right_current[1]=0;
            coefficient_right_current[0]=0;
        }
        //will be sliding windows to next frame
        detected_right = false;
        detected_left  = false;

        //To show just on window some calculations. Not related algoritms
        CalculateLineDistances(coefficient_left_current,coefficient_right_current);

    }
}


//Ekleme yapmayacak. CheckIsItLane() true ve false gelsede bu fonksiyona giriliyor.
//False geldiği durumda _recent değerlere ekleme yapmaması lazım.
//şuanda yapılan eklemeler kullanılmadığı için sorun yok.
void Lane::CalculateLineDistances(float* left, float* right){


    int temp_l_x_indis[3];
    int temp_r_x_indis[3];

    temp_l_x_indis[0]  = GetXValue(left,  y_size);
    temp_r_x_indis[0] = GetXValue(right, y_size);

    temp_l_x_indis[1]  = GetXValue(left,  (y_size*80) / 100);
    temp_r_x_indis[1] = GetXValue(right, (y_size*80) / 100);

    temp_l_x_indis[2]  = GetXValue(left,  (y_size*40) / 100);
    temp_r_x_indis[2] = GetXValue(right, (y_size*40) / 100);


    if(detected_right==true && detected_left==true){



        //Adding right & left lane x_indis
        left_lane_x_indis_current  = new int[3];
        right_lane_x_indis_current = new int[3];


        left_lane_x_indis_current[0]  = temp_l_x_indis[0];
        right_lane_x_indis_current[0] = temp_r_x_indis[0];

        left_lane_x_indis_current[1]  = temp_l_x_indis[1];
        right_lane_x_indis_current[1] = temp_r_x_indis[1];

        left_lane_x_indis_current[2]  =temp_l_x_indis[2];
        right_lane_x_indis_current[2] = temp_r_x_indis[2];

        //index increase
        ++buffer_lane_x_l;
        ++buffer_lane_x_r;


        //if index bigger than buffer then turn head
        buffer_lane_x_l %= buffer;
        buffer_lane_x_r %= buffer;

        //Note:First deleting the memory then adding on the current index

        //Adding right & left lane x_indis (recent)
        delete [] left_lane_x_indis_recent [ (buffer_lane_x_l) ];
        delete [] right_lane_x_indis_recent [ (buffer_lane_x_r) ];

        left_lane_x_indis_recent [ (buffer_lane_x_l) ] = left_lane_x_indis_current;
        right_lane_x_indis_recent [ (buffer_lane_x_r) ] = right_lane_x_indis_current;
    }
    else if (detected_right==true){

        right_lane_x_indis_current = new int[3];

        right_lane_x_indis_current[0] = temp_r_x_indis[0];
        right_lane_x_indis_current[1] = temp_r_x_indis[1];
        right_lane_x_indis_current[2] = temp_r_x_indis[2];

        //index increase
        ++buffer_lane_x_r;

        //if index bigger than buffer then turn head
        buffer_lane_x_r %= buffer;

        //Note:First deleting the memory then adding on the current index
        //Adding right lane x_indis (recent)
        delete [] right_lane_x_indis_recent [ (buffer_lane_x_r) ];

        right_lane_x_indis_recent [ (buffer_lane_x_r) ] = right_lane_x_indis_current;
    }
    else if (detected_left==true){

        //Adding right & left lane x_indis
        left_lane_x_indis_current  = new int[3];


        left_lane_x_indis_current[0]  = temp_l_x_indis[0];
        left_lane_x_indis_current[1]  = temp_l_x_indis[1];
        left_lane_x_indis_current[2]  =temp_l_x_indis[2];


        //index increase
        ++buffer_lane_x_l;
        //if index bigger than buffer then turn head
        buffer_lane_x_l %= buffer;

        //Note:First deleting the memory then adding on the current index
        //Adding right & left lane x_indis (recent)
        delete [] left_lane_x_indis_recent [ (buffer_lane_x_l) ];

        left_lane_x_indis_recent [ (buffer_lane_x_l) ] = left_lane_x_indis_current;
    }






    //Adding Distance between right - left lines
    distance_right_left_current    = new int[3];
    distance_right_left_current[0] = (temp_r_x_indis[0]-
                                      temp_l_x_indis[0]);
    distance_right_left_current[1] = (temp_r_x_indis[1]-
                                      temp_l_x_indis[1]);
    distance_right_left_current[2] = (temp_r_x_indis[2]-
                                      temp_l_x_indis[2]);


    //Adding Distance between center&lines
    distance_center_left_current  = new int[3];
    distance_right_center_current = new int[3];

    distance_center_left_current[0]  = ( image_center_x - temp_l_x_indis[0] );
    distance_right_center_current[0] = ( temp_r_x_indis[0] - image_center_x );

    distance_center_left_current[1]  = ( image_center_x - temp_l_x_indis[1] );
    distance_right_center_current[1] = ( temp_r_x_indis[1] - image_center_x );

    distance_center_left_current[2]  = ( image_center_x - temp_l_x_indis[2] );
    distance_right_center_current[2] = ( temp_r_x_indis[2] - image_center_x );





    //index increase
    ++buffer_lane_calculations;

    //if index bigger than buffer then turn head
    buffer_lane_calculations %= buffer;

    //Note:First deleting the memory then adding on the current index

    //Adding Distance between right - left lines (recent)
    delete [] distance_right_left_recent [ (buffer_lane_calculations) ];
    distance_right_left_recent [ (buffer_lane_calculations) ] = distance_right_left_current;

    //Adding Distance between center&lines (recent)
    delete [] distance_center_left_recent [ (buffer_lane_calculations) ];
    delete [] distance_right_center_recent [ (buffer_lane_calculations) ];
    distance_center_left_recent[ (buffer_lane_calculations) ] = distance_center_left_current;
    distance_right_center_recent[ (buffer_lane_calculations) ] = distance_right_center_current;


}


using namespace cv;



void generatePoints(int numPoints ,const float* left_fit, const float* right_fit, Point* left_line_points ,Point* right_line_points ,int xsize,int ysize, int poly_degree)
{
    
    int left_y=0; 
    int right_y=0;
     for(int col=0; col<xsize ;col++){
        left_y=0; 
        right_y=0;
        for (int i = 0; i < poly_degree+1; ++i)
        {
            left_y += left_fit[i] * std::pow(col, i);
            right_y += right_fit[i] *std::pow(col,i);

        }
        fprintf(stdout,"left_y : %d\n",left_y);
        fprintf(stdout, "left fit : %.2f %.2f %.2f\n",left_fit[0],left_fit[1],left_fit[2]);
        fprintf(stdout,"right_y : %d\n",right_y);
        fprintf(stdout, "right_fit : %.2f %.2f %.2f\n",right_fit[0],right_fit[1],right_fit[2]);


        left_line_points[col] = Point( left_y, col);
        right_line_points[col] = Point( right_y, col);
    }
    
}


void DrawLines(cv::Mat binary_warped, float* left_fit, float* right_fit, int xsize, int ysize, int poly_degree){
    Point left_line_points[1][xsize];
    Point right_line_points[1][xsize];
    // for(int col=0; col<xsize ;col++){
    //     left_line_points[0][col] = Point( (left_fit[0]) * pow(col,2) + (left_fit[1]) * col + (left_fit[2]), col);
    //     right_line_points[0][col] = Point( (right_fit[0]) * pow(col,2) + (right_fit[1]) * col + (right_fit[2]), col);
    // }
    generatePoints(xsize, left_fit, right_fit, left_line_points[0],right_line_points[0], xsize, ysize, poly_degree);
    const Point* ppt_left[1] = { left_line_points[0] };
    const Point* ppt_right[1] = { right_line_points[0] };
    int npt[] = { xsize };

    polylines(binary_warped, ppt_left, npt,1, 0, Scalar( 125, 0,0 ),4,8,0);
    polylines(binary_warped, ppt_right, npt, 1, 0, Scalar( 125,0, 0 ),4,8,0);

    imshow("Image",binary_warped);
    waitKey(1);
}


void polyfit(const std::vector<int>& x_inds, std::vector<int>& y_inds, int degree ,float* outputCoeffs)
{
    cv::Mat A(x_inds.size(), degree + 1, CV_32F);
    cv::Mat b(y_inds.size(), 1, CV_32F);

    // Fill A and b matrices
    for (int i = 0; i < x_inds.size(); ++i)
    {
        float x = x_inds[i];
        float y = y_inds[i];

        for (int j = 0; j <= degree; ++j)
        {
            A.at<float>(i, j) = std::pow(x, j);
        }
        b.at<float>(i, 0) = y;
    }

    // Solve the system
    cv::Mat coeffs;

    cv::solve(A, b, coeffs, cv::DECOMP_SVD);
    fprintf(stdout, "right_fit : %.2f %.2f %.2f\n",outputCoeffs[0],outputCoeffs[1],outputCoeffs[2]);


}
void stride_window(Mat& binary_warped , int xsize, int ysize, int poly_degree){
     
    float left_fit[poly_degree], right_fit[poly_degree]; // Left and Right line

    // Set the width of the windows +/- margin
    int margin = 20;

    // Choose the number of sliding windows
    int nwindows = 18;

    // Set minimum number of pixels found to recenter window
    int minpix = 50;

    // Create empty lists to receive left and right lane pixel indices
    vector<int> left_lane_x_inds;
    vector<int> left_lane_y_inds;
    vector<int> right_lane_x_inds;
    vector<int> right_lane_y_inds;

    // fprintf(stdout, "Sliding Window make histogram\n");
    // Take a histogram of the bottom half of the image
    cv::Mat take_half= binary_warped.rowRange(binary_warped.rows/2,binary_warped.rows);
    cv::Mat histogram(cv::Mat::zeros(1, take_half.cols, CV_32F));

    // Histogram
    cv::reduce(take_half, histogram, 0, 0, CV_32F); // REDUCE_SUM flag equals to 0.

    // Find the peak of the left and right halves of the histogram
    // These will be the starting point for the left and right lines
    Mat left_histogram = histogram.colRange(0, histogram.cols/2);
    Mat right_histogram = histogram.colRange(histogram.cols/2, histogram.cols);

    //finding min-max location for left line
    double min_val_l, max_val_l;
    Point  max_loc_l;
    Point  min_loc_l;
    cv::minMaxLoc(left_histogram, &min_val_l, &max_val_l, &min_loc_l, &max_loc_l);

    //finding min-max location for right line
    double min_val_r, max_val_r;
    Point  max_loc_r;
    Point  min_loc_r;
    cv::minMaxLoc(right_histogram, &min_val_r, &max_val_r, &min_loc_r, &max_loc_r);

    // Set the height of windows
    int window_height = binary_warped.rows / nwindows;

    fprintf(stdout, "find current positions\n");
    // Current positions to be updated for each window
    int l_current = max_loc_l.x;
    int r_current = max_loc_r.x + histogram.cols/2;
    // Set the height of windows



    // Create an output image to draw on and  visualize the result //for debug
    fprintf(stdout, "create out_img\n");
    Mat out_img=binary_warped.clone();

    // Step through the windows one by one
    for(int window=0; window < nwindows; window++){

        // Identify window boundaries in x and y (and right and left)
        int win_y_low = binary_warped.rows - (window+1) * window_height; // Top edge
        int win_y_high = binary_warped.rows - window * window_height;      // Bottom edge

        int win_xleft_low = l_current - margin;     //LEFT minumun (left edge)
        int win_xleft_high = l_current + margin;    //Left maximum (right edge)

        int win_xright_low = r_current - margin;    //Right maximum (left edge)
        int win_xright_high = r_current + margin;   //Right minumum (right edge)

        // fprintf(stdout, "draw windows\n");
        // Debug. Draw the windows on the visualization image
        // if (debug){

            // a: Left Rectangle left Top Corner
            // b: Left Rectangle right Bottom Corner
        Point2d a(win_xleft_low, win_y_low), b(win_xleft_high, win_y_high);
        rectangle(out_img, a, b, cv::Scalar(255), 2); //a and b opposite direction //Drawing rectangle on out_img

        // Right Rectangle left Top Corner.
        a.x = win_xright_low;
        a.y = win_y_low;
        // Right Rectangle right Bottom Corner
        b.x = win_xright_high;
        b.y = win_y_high;
        rectangle(out_img, a, b, cv::Scalar(255), 2); // a and b opposite direction //Drawing rectangle on out_img

        cv::namedWindow( "box", cv::WINDOW_NORMAL);
        imshow("box", out_img);
        cv::waitKey(1);
        // cv::destroyAllWindows();
        // }

        Mat Roi;

        // Process Left Window
        if(win_xleft_low < 0){
            win_xleft_low=0;
        }

        if(win_xleft_high > xsize){
            win_xleft_high=xsize;
        }

        Rect RecLeft(win_xleft_low, win_y_low, win_xleft_high-win_xleft_low, window_height);
        Roi = binary_warped(RecLeft);   // ROI: Region of Interest
        std::vector<cv::Point2i> nonzero;

        findNonZero(Roi, nonzero);  // Find only white areas
        int sum = 0;
        for (std::vector<cv::Point2i>::iterator it = nonzero.begin(); it != nonzero.end(); ++it){
            int x;
            left_lane_y_inds.push_back((*it).y + win_y_low);    // Pushing Y point of white point in ROI
            x = (*it).x + win_xleft_low;
            left_lane_x_inds.push_back(x);                      // Pushing X point of white point in ROI
            sum += x;                                           // Sum of X indices of white point
        }

        // Evaluate if white points are sufficient
        if(nonzero.size() > minpix){
            l_current = int(sum / nonzero.size()); // Average of X indices of white points

            // Stop when the line goes to out of image
            if (l_current - margin <= 0){
                break; 
            }
        }

        // Process Right Window
        if(win_xright_low < 0){
            win_xright_low=0;
        }
        if(win_xright_high> xsize){
            win_xright_high=xsize;
        }

        Rect RecRight(win_xright_low, win_y_low, win_xright_high-win_xright_low, window_height);
        Roi = binary_warped(RecRight); // ROI: Region of Interest

        nonzero.clear(); // Clear vector
        findNonZero(Roi, nonzero); // Find only white areas
        sum = 0;
        for (std::vector<cv::Point2i>::iterator it = nonzero.begin() ; it != nonzero.end(); ++it){
            int x;
            right_lane_y_inds.push_back((*it).y + win_y_low);   // Pushing Y point of white point in ROI
            x = (*it).x + win_xright_low;
            right_lane_x_inds.push_back(x);                     // Pushing X point of white point in ROI
            sum += x;                                           // Sum of X indices of white point

        }

        // Evaluate if white points are sufficient
        if(nonzero.size() > minpix){
            r_current = int(sum / nonzero.size()); // Average of X indices of white points

            // Stop when the line goes to out of image
            if (r_current + margin >= xsize){

                break;
            }
        }
    }

//TODO
    // If white points in the windows are so small.
    int thres = minpix;


    if( left_lane_y_inds.size() > thres && right_lane_y_inds.size() > thres){
        polyfit(left_lane_y_inds,left_lane_x_inds, poly_degree,left_fit);
        polyfit(right_lane_y_inds,right_lane_x_inds, poly_degree,right_fit);    
        DrawLines(binary_warped, left_fit, right_fit,xsize, ysize, poly_degree);

    }

}



int houghline_detect(Mat& binary_warped){



    double rho = 1;
    double theta = CV_PI/180;
    int threshold = 200;
    double minLineLength = 150;
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


    cv::imshow("Houglines", cdst);


    double sum_angle = 0.0;
    int count=0;
    vector<float> theta_list;


    for (size_t i = 0; i < lines.size(); i++) {
        
        if((lines[i][2] -lines[i][0])==0){
            continue;
        }
        float theta;
   
        
         theta = -(lines[i][3] -lines[i][1])/(lines[i][2] -lines[i][0]);
       

        double angle =atan(theta);
        
        if(angle <0){angle +=CV_PI;}
        theta_list.push_back(angle);
        fprintf(stderr,"amgle : %.2f\n",angle);
        fprintf(stderr, "cosine :%.2f\n",cos(angle) );
        // Convert theta to degrees
        }
    
    fprintf(stderr,"make historgram numberr \n" );
     if (theta_list.size() ==0 ){
        return 0;
     }

    FloatHistogram histogram(theta_list, 20);

    // std::cout << "Original Histogram:" << std::endl;vg_angle/(CV_PI))*180)
    // histogram.printHistogram();

    // SlidingWindow(binary_warped);
     double avg_angle;
    if (theta_list.size() > 0) {
        avg_angle = histogram.getMaxHistogramData();
        std::cout << "Average line angle: " << avg_angle << " degrees" << std::endl;
    } else {
        std::cout << "No lines detected" << std::endl;
        return 0;
    }
    
    int new_angle= -(int((avg_angle/(CV_PI))*180)-90); 

     
    
     return new_angle;


}

