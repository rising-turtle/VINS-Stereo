#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;

const int NUM_OF_CAM = 2; 
const int STEREO = 1;

extern std::string IMAGE1_TOPIC;
extern std::string IMAGE2_TOPIC; 
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern int FLOW_BACK;
extern Eigen::Matrix3d E; // essential matrix 

void readParameters(ros::NodeHandle &n);
