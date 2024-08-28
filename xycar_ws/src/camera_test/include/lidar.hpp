#ifndef lidar_hpp
#define lidar_hpp

#include<algorithm>
#include <opencv2/opencv.hpp>



#define LIDAR_SIZE 505
#define BOUNDARY_X 32
#define BOUNDARY_Y 100

using namespace std;
float find_boundary_max(int r , int x, int y);

vector<int> find_boundary_point(float* lidar_point);

int find_angle(const vector<int>& boundary_points,float* lidar_point );




















#endif