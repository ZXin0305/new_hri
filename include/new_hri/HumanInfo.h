#pragma once
#include<iostream>
#include<vector>
#include<string>
#include<ros/ros.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
// 用来接收人员的姿态信息
#include<new_hri/Human.h>
#include<new_hri/HumanList.h>
#include<new_hri/PointCoors.h>
// #include<new_hri/send2Robot.h>
// #include<new_hri/IDWithAction.h>   //之前是想用这个来发布人员的id和身份信息，但是不太行，就没有用了
#include<std_msgs/Int32MultiArray.h>  //用来存储并发布人员的身份id和动作信息 --> [human_id, human_action; human_id, human_action; .....]
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<map>

#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>   //提取图像的头文件
using namespace std;

const int N = 15;  //joint num
const int MAX_MEO = 8;
const int pairNum = 14;
int bodyEages[pairNum][2] = {
    {0,1}, {0,2}, {0,9}, {0,3}, {3,4}, {4,5},
    {9,10}, {10,11}, {2,6}, {6,7},
    {7,8}, {2,12}, {12,13}, {13,14}
};

Eigen::Matrix<double,3,1> world_xyz;
double min_x = 0.01;
double min_y = 0.01;
double max_x = 0.2;
double max_y = 0.2;
double z_thres = 0.12;
// vector<string> human_joint_name{"neck", "root", "lshoulder", "lelbow", "lwrist","rshoulder", "relbow", "rwrist"};

struct KeyPoint
{
    double x;
    double y;
    double z;
};

struct HumanWithIdAction{
    int action;
    int human_id;
    vector<KeyPoint> point;
    // KeyPoint *point = new KeyPoint[N];
};

class PointPub{
public:
    PointPub(){
        this->points.header.frame_id = this->world_name;
        this->points.header.stamp = ros::Time::now();
        this->points.ns = this->name_space;
        this->points.action = visualization_msgs::Marker::ADD;
        this->points.pose.orientation.w = 1.0;
        this->points.pose.orientation.x = 0.0;
        this->points.pose.orientation.y = 0.0;
        this->points.pose.orientation.z = 0.0;
        this->points.id = 0;
        this->points.type = visualization_msgs::Marker::POINTS;

        this->points.scale.x = 0.03;
        this->points.scale.y = 0.03;
        this->points.scale.z = 0.03;
        this->points.color.r = 1.0;
        this->points.color.a = 1.0;
        this->points.lifetime = ros::Duration(1);
    }

    void store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList);
public:
    visualization_msgs::Marker points;
private:
    string world_name = "base";
    string name_space = "marker_node";
};


// 这个是将人体的姿态信息连成线并发布到Rviz空间中，是一种视觉反馈
class LinePub
{
public:
    LinePub(){
        this->line.header.frame_id = this->world_name;
        this->line.header.stamp = ros::Time::now();
        this->line.ns = this->name_space;
        this->line.action = visualization_msgs::Marker::ADD;
        this->line.pose.orientation.w = 1.0;
        this->line.pose.orientation.x = 0.0;
        this->line.pose.orientation.y = 0.0;
        this->line.pose.orientation.z = 0.0;

        this->line.id = 1;
        this->line.type = visualization_msgs::Marker::LINE_LIST;
        this->line.scale.x = 0.01;
        this->line.scale.y = 0.01;
        this->line.scale.z = 0.01;
        //color
        this->line.color.g = 1.0;
        this->line.color.a = 1.0;
        this->line.color.r = 0.8;
        this->line.color.b = 1.0;
        this->line.lifetime = ros::Duration(1);
    }

    void store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList);
public:
    visualization_msgs::Marker line;
private:
    string world_name = "marker_0";  //marker_0
    string name_space = "marker_node";  
};

class Human{
public:
    Human(string camera_name):camera_name(camera_name)
    {
        this->human_sub = nh.subscribe(this->_pose_topic, 5, &Human::human_pose_callback, this);
        this->human_idAction_pub = nh.advertise<std_msgs::Int32MultiArray>(this->_human_id_action, 2);
        point_pub = new PointPub;
        line_pub = new LinePub;
        tf_listener();
    }

    void get_human_pose(vector<HumanWithIdAction>& human_pose_with_id_action);
    void change_pose_format(vector<vector<Eigen::Matrix<double,3,1>>>& point_list);
    void pub_human_info(vector<vector<Eigen::Matrix<double,3,1>>>& point_list);
    void human_pose_callback(const new_hri::HumanList& human_list);
    void run();
    void tf_listener();
    void cal_cross_point();  //这个idx指的是for 循环中的人员的顺序
    bool intersectionLinePlane(Eigen::Vector3f &p1, Eigen::Vector3f p2, const Eigen::Vector4f &plane, Eigen::Vector3f &crossP);

    ~Human(){
        delete point_pub;
        delete line_pub;
    }
public:
    //发布人的关节点信息用
    vector<int32_t> tmp_id_action;
    std_msgs::Int32MultiArray human_id_with_action;
    tf::Transform current_joint;
    string joint_name;
    double X, Y, Z;
    Eigen::Matrix3d cam2world;
    Eigen::Vector3d trans;

    //两个空间点 实验２
    Eigen::Vector3f point_1;
    Eigen::Vector3f point_2;
    Eigen::Vector4f plane;
    Eigen::Vector3f cross_point; //待求解的点
    tf::StampedTransform task_point;

private:
    // image_transport::ImageTransport it;
    // image_transport::Subscriber image_sub;

    vector<HumanWithIdAction> human_points_vector;
    ros::NodeHandle nh;
    ros::Subscriber human_sub;
    ros::Publisher human_idAction_pub;
    tf::TransformBroadcaster human_info_br; //用这个发布人体的信息
    vector<tf::Transform> human_info_trans;  //保存点的信息...和　
    string _pose_topic = "/pub_human"; //前面的是接收从python端发布的关节点话题，后面的是发布出去当前的人员的id和动作
    string _human_id_action = "/human_id_action"; //发布到机器人端
    string camera_name;
    PointPub *point_pub;
    LinePub *line_pub;
};