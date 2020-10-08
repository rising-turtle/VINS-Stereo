#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
double distance(cv::Point2f pt1, cv::Point2f pt2);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1); 
    void trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1); 

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(vector<string> &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);

    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);

    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap);
    cv::Mat getTrackImage();
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    // vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    // vector<cv::Point2f> prev_un_pts, cur_un_pts;
    // vector<cv::Point2f> pts_velocity;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;

    // vector<int> ids;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    // map<int, cv::Point2f> cur_un_pts_map;
    // map<int, cv::Point2f> prev_un_pts_map;

    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;

    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
    int stereo_cam; 
    bool hasPrediction;
    vector<cv::Point2f> predict_pts;
};
