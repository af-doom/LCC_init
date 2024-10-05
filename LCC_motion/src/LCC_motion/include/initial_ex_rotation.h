#pragma once 

#include <vector>
#include "parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <ros/console.h>
#include<ros/ros.h>
/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
	InitialEXRotation(); 
    void Reset();    
     bool CalibrationTime(const double frame_time, vector<pair<Vector3d, Vector3d>> &corres,vector<Eigen::Vector3d>& angular_velocity_buf, double &tdtime);
    bool CalibrationExPosition(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Vector3d delta_p_imu, Matrix3d &calib_ric_result,Vector3d &calib_tic_result);
    Eigen::Matrix3d  CalibrationExRotation( Quaterniond delta_q_lidar, Quaterniond delta_q_camera );
    bool No_CalibrationExRotation(bool judge_motion,vector<pair<Vector3d, Vector3d> > corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
    Eigen::Vector4d VI_data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num,  Eigen::Vector3d &lidar_omg, int &orig_odom_freq,
                                       int &cut_frame_num);
                                       
    void clear() ;
    void printProgress(double percentage, int axis_ascii);
    // MatrixXd Jaco_rot(30000, 3);
   
    // bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
private:
	Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
                 
    int frame_count;
   int frame_count_;
    vector< Matrix3d > Rc;
    vector< Matrix3d > Rimu;
    vector< Matrix3d > Rc_g;
    Matrix3d ric;

     int frameR_count;
    int frameT_count;
    int frameRT_count;
    double cov_thresh;

    vector< Vector3d > Wc_t;
    vector< Vector3d > Wimu;
    vector< Vector3d > Wimu_t;
    vector<vector<Vector3d> > Wimu_t_buf;
    vector< double > Frame_time;
        vector< Vector3d > tc;
    vector< Vector3d > timu;
    public:
    
    Vector3d tic;
    Matrix3d ideal_ric;
    bool cflag;
};


