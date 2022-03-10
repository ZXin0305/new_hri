#pragma once
#include<iostream>
#include<sstream>
#include<algorithm>
#include<string>
/*        opencv & aruco         */
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/aruco.hpp>
#include<opencv2/aruco/dictionary.hpp>
/*        ros         */
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
/*        Eigen         */
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
/*        toCv          */
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>

#include<std_msgs/Int32MultiArray.h>  //用来发布marker的id信息

using namespace std;
using namespace cv;

class Object
{
public:
    Object(string camera_topic, string camera_param_path):_it(_nh)
    {
        _axis_trans_y = -0.15;
        _axis_trans_x = _axis_trans_z = 0.0;
        _W = 0.375;
        _X = 0.925;
        _Y = -0.011;
        _Z = 0.062;
        this->_camera_topic = camera_topic;
        this->_camera_param_path = camera_param_path;
        this->load_camera_param();
        //订阅图像话题
        _image_sub = _it.subscribe(this->_camera_topic,1,&Object::img_callback,this);
        _marker_id_pub = _nh.advertise<std_msgs::Int32MultiArray>("marker_detect",2);
        
    }

    void load_camera_param();
    void img_callback(const sensor_msgs::ImageConstPtr& msg);
    void run();
    void locate_marker();
    void sendMarkerTF(cv::Vec3d& marker_trans,cv::Vec3d& marker_rot,int &id);
    void sendMarkerID();

private:
    string _camera_topic;  //订阅图像
    string _camera_param_path; //相机内参文件路径
    cv::Mat _camera_matrix;
    cv::Mat _camera_dist;
    
    //image
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    cv::Mat _img;
    //marker  DICT_4X4_50
    cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);  //到时候不一定用这样的
    vector<int> _markerIDs;
    vector<vector<cv::Point2f>> _markerCorners;
    vector<cv::Vec3d> _rvecs;
    vector<cv::Vec3d> _tvecs;

    //直接将得到的空间位置（在某个轴上会具有一定的位移），
    //然后配合固定的旋转就可以发布出去就可以
    double _axis_trans_y, _axis_trans_x, _axis_trans_z;
    double _W, _X, _Y, _Z;

    std_msgs::Int32MultiArray _marker_id_list;
    vector<int32_t> _tmp_marker_vec;
    ros::Publisher _marker_id_pub;
};


