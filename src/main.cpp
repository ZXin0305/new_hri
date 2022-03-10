#include<new_hri/HumanInfo.h>

void Human::run(){
    ros::Rate rate(60);
    vector<vector<Eigen::Matrix<double,3,1>>> pointList;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/m",100);
    cout<<"begin human thread .."<<endl;
    while(ros::ok()){
        ros::spinOnce();
        if(this->human_points_vector.size() == 0)
            ROS_INFO("no human");
        else{
            pointList.clear();
            this->change_pose_format(pointList);  // cam --> to world
            this->pub_human_info(pointList);   //发布人体的关节点和其他的tf数据, 被机器人那边进行了接收
            // point_pub->store_points(pointList);
            line_pub->store_points(pointList);   //发布关节点的坐标到rviz空间中,到时候这里的坐标会已经转化到世界坐标系中了
            // pub.publish(point_pub->points);
            pub.publish(line_pub->line);  //发布到Rviz中进行可视化
            ROS_INFO("have send human info ..");
        }

        rate.sleep();   
    }    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sub_human_pose");
    ros::NodeHandle nh;
    string camera_name = "camera_base_1";
    Human human(camera_name);
    human.run();
    // PointPub point_pub;
    // LinePub line_pub;

    // ros::Rate rate(30);
    
    // vector<vector<Eigen::Matrix<double,3,1>>> pointList;
    // ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("m",100);
    // cout<<"begin human thread .."<<endl;
    // while(ros::ok()){
    //     ros::spinOnce();
    //     if(human.human_points_vector.size()==0)
    //         ROS_INFO("no human");
    //     else{
    //         ROS_INFO("human num %d", human.human_points_vector.size());
    //         pointList.clear();
    //         human.pub_human_info();   //发布人体的关节点和其他的数据, 被机器人那边进行了接收
    //         human.change_pose_format(pointList);
    //         line_pub.store_points(pointList);   //到时候这里的坐标会已经转化到世界坐标系中了
    //         pub.publish(line_pub.line);  //发布到Rviz中进行可视化
    //         ROS_INFO("have send human info ..");
    //     }
    //     rate.sleep();   
    // }
}
