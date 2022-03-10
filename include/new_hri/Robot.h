#pragma once
#include<boost/scoped_ptr.hpp>
// ------- kdl运动学库 --------
#include<kdl/chain.hpp>
#include<kdl/jacobian.hpp>
#include<kdl/chainjnttojacsolver.hpp>
#include<kdl/jntarray.hpp>
#include<kdl_parser/kdl_parser.hpp>
// ------- 逆运动学库 --------
#include<trac_ik/trac_ik.hpp>
// ------- ros -------
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf_conversions/tf_kdl.h>
// ------- ur5 -------
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>
#include<pthread.h>
#include<iostream>
#include<cmath>
#include<vector>
#include<string>
// 用来接收人的信息
#include<std_msgs/Int32MultiArray.h>
#include<algorithm>
#include<cmath>
#include<ctime>

//记录数据
#include<ostream>
#include<fstream>

using namespace std;

const int JOINT_NUM=6;  //这个是机器人的关节数
const int PUB_RATE=20;  //机器人运动指令发布的频率
const float TIME_STEP=1.0 / PUB_RATE;
vector<string> human_action{"walking", "stand", "operate", "shake", "T-pose"};

// init some params
bool JointState_available;  //能够获取机器人的关节状态
KDL::JntArray q(JOINT_NUM); //当前的关节状态的向量
KDL::JntArray q_desired(JOINT_NUM); //期望的关节状态的向量
KDL::Frame correct_pose, desired_pose;

std::vector<std::string> joint_names(JOINT_NUM);   //机器人的关节名称
std::vector<double> joint_angles(JOINT_NUM);      //机器人的关节角度
std::vector<double> joint_speed(JOINT_NUM);       //机器人的关节速度

string urdf_param = "/robot_description";  //ur5的机器人模型参数
string base_name = "base";
string tip_name = "tool0";

//这几个只是关于夹取点的
string mode1 = "track_init";
string mode2 = "track_cap";
string mode3 = "track_place";
string mode4 = "track_obj";

const float delay_time = 2.0;  //这个到时候是等待机械臂的固定点的时间

class Robot{
public:
    Robot(){
        string robot_topic1 = "/ur_driver/URScript";
        string robot_topic2 = "/joint_states";
        this->init();   //这个是机械臂的一些参数
        ur_command_publisher = nh.advertise<std_msgs::String>(robot_topic1, 1);
        ur_joint_publisher = nh.advertise<sensor_msgs::JointState>(robot_topic2, 1);
        joint_state_sub = nh.subscribe(robot_topic2, 3, &Robot::jointStateCallback,this);
        
        //这里的回调函数，订阅human id和action并且监听人体的关节点信息
        human_id_action_sub = nh.subscribe(_human_id_action, 3, &Robot::humanIdActionCallback, this);
        //订阅物件marker的id
        // _object_id_sub = nh.subscribe(_object_id, 3, &objectIdCallback, this);
    }

    void init(); 
    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void* goal_rectify_thread(void *arg);
    void sendCmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point);  //发布控制指令
    bool isToPlace();   //判断机器人是否到达预定的位置 后面未使用
    void adjust_speed(vector<double>& speed_vector);   //根据手腕和末端的距离进行速度的调整
    double cal_distance(tf::StampedTransform& tf_point, string& robot_joint_name);  //optional -->　在过程中判断人体和机器人的距离，调整速度
    void sort_distance();
    //监听交互中的点　..
    //但是这里的时候使用固定的点
    void listener_object(); //监听物件的marker
    void listener_point();  //监听机器人的初始位置和任务的抓取点
    tf::Vector3 listener_robot_point(string& robot_joint_name);  //监听机器人自身的点的tf信息

    //监听人体的关节点信息和id action
    void humanIdActionCallback(const std_msgs::Int32MultiArray& id_action_array);
    // void objectIdCallback(const std_msgs::Int32MultiArray& object_id_array);

    //判断是否达到该位置
    bool judge(tf::StampedTransform& tf_point);

    void test();
    void track();  //实验1,人机安全实验

    bool listen_reach_task(); //实验2
    int listen_action();

    void reach_point(string case_);

    int get_correct_target_size();

public:
    //接收人体的动作和id信息
    vector<int32_t> tmp_id_action, _object_id;   //id 和　action
    int human_num, task_num;  //human_num是指当前场景中的人员数量  task_num是指当前场景中检测到的物块的tf信息
    vector<tf::StampedTransform> all_human_joint_tf, all_object_tf;

    //实验１
    string mode;
    bool toInitPoint; //到达初始点的信号
    bool toCapPoint; //到达过渡点的信号
    bool toPlacePoint; //到达放置点的信号
    bool toObjPoint; //到达预计的物件的位置
    int obj_num;
    int obj_flag;
    string stage;

    //实验2
    tf::StampedTransform task_point;

    //调整速度
    double _closest_dis;
    int _closest_idx;
    vector<double> distance_vec;
    double kD;
    tf::StampedTransform base_human_dis, base_tool_dis;
    double _max_trend_thres = 2.0;  // M_PI / 2

    //上一步的机器人的位置
    tf::Vector3 _base_tool_vector, _last_base_tool_vector;
    int _start_step = 0;
    int _update_rate = 2;
    int _update_flag = 0;
    //判定机器人是不是在和人进行协作
    bool _robot_is_cooperate = false;
    //数据记录路径
    string _data_path = "/home/zx/zx_ws/src/new_hri/data/data.txt";
    int _test_num = 0;

private:

    string _joint_name_tool0 = "tool0", _joint_name_controller = "tool0_controller";
    pthread_t rectify_thread;
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_br;

    //用来订阅human发布的action和id，但是这是个列表,
    //需要利用这个得到人员的id从而去监听对应的关节点的信息
    ros::Subscriber human_id_action_sub, _object_id_sub;
    string _human_id_action = "/human_id_action";

    // /ur_driver/URScript：
    // 话题名称，类型是std_msgs::String，
    // 允许我们向该话题发布URScript脚本命令
    ros::Publisher ur_command_publisher;  

    // /joint_states：
    // 话题名称，类型是：sensor_msgs::JointState，
    // 实现机械臂的运动，就是向话题joint_states添加有关关节的消息
    ros::Publisher ur_joint_publisher;
    ros::Subscriber joint_state_sub; //实时订阅机器人关节角度的信息
    // string urdf_param = "/robot_description";  //ur5的机器人模型参数
    // string base_name = "base";
    // string tip_name = "tool0";
    // TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    
    std_msgs::String command_msg;
    double Kp = 2.0;
    double Td = 0.2;

    //设置不同的阈值调整速度
    double min_thres = 0.2;
    double max_thres = 0.4;

    double min_speed = 0.5;
    double middle_speed = 1.0;
    double max_speed = 1.5;

    string urdf_param = "/robot_description";  //ur5的机器人模型参数
    string base_name = "base";
    string tip_name = "tool0";

    bool toPlace , update;

    //相关的点的信息
    tf::StampedTransform cap_point, init_point, place_point;
    bool tf_available = false;
    tf::StampedTransform transform_correct, transform_target;
    vector<tf::StampedTransform> correct_target, cap_init;  //前面的是到时候的物件的矫正点和抓取点　后面的是机器人的初始点和夹取物件的地方
};