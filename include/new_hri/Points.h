#pragma once
#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<cmath>
#include<queue>
#include<std_msgs/String.h>

using namespace std;

class Points{
public:
    Points(){
        this->init_points();
    }

    void init_points();
    void pub_points();

private:
    ros::NodeHandle nh;
    vector<tf::Transform> cap_init, obj_target;
    tf::Transform tmp, _task_joint;
    tf::TransformBroadcaster br;
    tf::TransformListener tf_listen;
};