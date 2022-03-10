#include<new_hri/Points.h>

void Points::init_points(){
    //初始点，过渡点，放置点
    //都是相较于marker_0的
    tf::Quaternion q_init(-0.6898,0.7235,-0.0054,-0.0257);  //初始点  X,Y,Z,W 
    q_init = q_init.normalize();
    tmp.setOrigin(tf::Vector3(-0.3,-0.2,0.55));
    tmp.setRotation(q_init);
    cap_init.push_back(tmp);

    tf::Quaternion q_cap(0.999,-0.0277,-0.0203,-0.00078);  //过渡点
    q_cap = q_cap.normalize();
    tmp.setOrigin(tf::Vector3(-0.806,0.209,0.35));
    tmp.setRotation(q_cap);
    cap_init.push_back(tmp);

    tf::Quaternion q_place(-0.6934,0.72045,0.000934,-0.00456);  //放置点
    q_place = q_place.normalize();
    tmp.setOrigin(tf::Vector3(-0.025,-0.20,0.55));
    tmp.setRotation(q_place);     
    cap_init.push_back(tmp);

/* ----------------------------------------------------------------------------- */
    //物件的固定抓取位置
    //６个物件
    tf::Quaternion q_obj0_cor(0.9996, -0.0143, -0.021646, 0.0139);  //obj0 correct
    q_obj0_cor = q_obj0_cor.normalize();
    tmp.setOrigin(tf::Vector3(-0.8,0.3,0.2));
    tmp.setRotation(q_obj0_cor);     
    obj_target.push_back(tmp);

    tf::Quaternion q_obj0_tar(0.999, -0.0243, -0.0083, -0.00389);  //obj0 target 
    q_obj0_tar = q_obj0_tar.normalize();
    tmp.setOrigin(tf::Vector3(-0.8,0.3,0.1));
    tmp.setRotation(q_obj0_tar);     
    obj_target.push_back(tmp);

    // tf::Quaternion q_obj1_cor(0.999, -0.0243, -0.0083, -0.0039);  //obj1 correct
    // q_obj1_cor = q_obj1_cor.normalize();
    // tmp.setOrigin(tf::Vector3(-1.01, 0.3257, 0.04));
    // tmp.setRotation(q_obj1_cor);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj1_tar(0.999, -0.0243, -0.0083, -0.0039);  //obj1 target
    // q_obj1_tar = q_obj1_tar.normalize();
    // tmp.setOrigin(tf::Vector3(-1.01, 0.3257, 0.04));
    // tmp.setRotation(q_obj1_tar);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj2_cor(0.714, -0.7, -0.002, 0.0004);  //obj2 correct
    // q_obj2_cor = q_obj2_cor.normalize();
    // tmp.setOrigin(tf::Vector3(-0.125, -0.120, 0.264));
    // tmp.setRotation(q_obj2_cor);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj2_tar(0.714, -0.7, -0.002, 0.0004);  //obj2 target
    // q_obj2_tar = q_obj2_tar.normalize();
    // tmp.setOrigin(tf::Vector3(-0.125, -0.120, 0.264));
    // tmp.setRotation(q_obj2_tar);     
    // obj_target.push_back(tmp); 

    //机械臂抓取点
    tf::Quaternion task_joint_quaternion(-0.6898,0.7235,-0.0054,-0.0257);
    task_joint_quaternion = task_joint_quaternion.normalize();
    _task_joint.setOrigin(tf::Vector3(-0.10,-0.23,0.35));
    _task_joint.setRotation(task_joint_quaternion);

}


void Points::pub_points(){
    // ----------------------------------- //
    // 固定的过渡点、初始点、放置点
    br.sendTransform(tf::StampedTransform(this->cap_init[0], ros::Time::now(), "marker_0", "init_point"));
    br.sendTransform(tf::StampedTransform(this->cap_init[1], ros::Time::now(), "marker_0", "cap_point"));
    br.sendTransform(tf::StampedTransform(this->cap_init[2], ros::Time::now(), "marker_0", "place_point"));

    // ----------------------------------- //
    //　物件
    br.sendTransform(tf::StampedTransform(this->obj_target[0], ros::Time::now(), "marker_0", "obj0_correct"));
    br.sendTransform(tf::StampedTransform(this->obj_target[1], ros::Time::now(), "marker_0", "obj0_target"));
    // br.sendTransform(tf::StampedTransform(this->obj_target[2], ros::Time::now(), "marker_0", "obj1_correct"));
    // br.sendTransform(tf::StampedTransform(this->obj_target[3], ros::Time::now(), "marker_0", "obj1_target"));
    // br.sendTransform(tf::StampedTransform(this->obj_target[4], ros::Time::now(), "marker_0", "obj2_correct"));
    // br.sendTransform(tf::StampedTransform(this->obj_target[5], ros::Time::now(), "marker_0", "obj2_target"));

    //　机械臂协助点
    br.sendTransform(tf::StampedTransform(this->_task_joint, ros::Time::now(), "marker_0", "task_point"));
    
    ROS_INFO("pub points ..");
}