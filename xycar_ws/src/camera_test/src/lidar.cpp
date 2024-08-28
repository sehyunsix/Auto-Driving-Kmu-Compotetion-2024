#include "lidar.hpp"
#include <vector>
#include<stdio.h>
#include <iostream>
#include <map>
#include "ros/ros.h"
#include<algorithm>

using namespace std;

double indextotheta(int x){
    float delta = CV_PI/250;
    if( x<504 && x>325){return (x-325)*delta;}
    if( x<125 && x>0){return (x+125)*delta;}
    return -1;

}

float find_boundary_max(int r , int boundary_x, int boundary_y){
    double theta=  indextotheta(r);
    if(theta == -1){return -1;}
    float y = min(abs(sin(theta)), abs(boundary_y/1.0));
    float x = min(abs(cos(theta)), abs(boundary_x/1.0));
    return sqrt(pow(x,2)+pow(y,2));
}

vector<int> find_boundary_point(float* lidar_point){
    vector<int> boundary_point_list;
    for (int i=0; i<LIDAR_SIZE; i++){
        if(lidar_point[i] >0){
            int  max_r = find_boundary_max(i,BOUNDARY_X,BOUNDARY_Y);
            if(lidar_point[i]<max_r){
                boundary_point_list.push_back(i);
            }
            boundary_point_list.push_back(max_r);
        }
    }
    // fprintf(stderr,"boundary_point_list : %d",boundary_point_list.size());
    
    return  boundary_point_list;
}

int find_angle(const vector<int>& boundary_point_list, float* lidar_point){

    int min = 500;
    int min_index= 0;
    for(auto &it : boundary_point_list){
     
           if(min > lidar_point[it] && min<0.01){
                min = lidar_point[it];
                min_index = it;
           } 
    }

    float theta = indextotheta(min_index);
    if(theta != -1);
    {  return 0;}


    theta = -(int(theta*180/CV_PI)-90);
    fprintf(stderr,"theta %.2f",theta);

    return theta;

};

