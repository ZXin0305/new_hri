#include<new_hri/HumanInfo.h>

void Human::human_pose_callback(const new_hri::HumanList& human_list){
    int human_num = human_list.human_list.size();
    this->human_points_vector.clear();  //这里要先将之前的数据清除，不然关节点会一直进行累加，这样就没什么问题了
    for(int i=0; i<human_num; ++i){
        int current_human_id = human_list.human_list[i].human_id;
        int current_human_action = human_list.human_list[i].action;
        auto human_points = human_list.human_list[i].body_points;  //当前这个人的关节点信息

        HumanWithIdAction hwia;
        hwia.action = current_human_action;
        hwia.human_id = current_human_id;

        for(int j=0; j<human_points.size(); ++j){
            
            KeyPoint current_point;
            current_point.x = human_points[j].x;
            current_point.y = human_points[j].y;
            current_point.z = human_points[j].z;
            
            hwia.point.push_back(current_point);  //
        }
        this->human_points_vector.push_back(hwia);
    }
}

void Human::get_human_pose(vector<HumanWithIdAction>& human_pose_with_id_action){
    human_pose_with_id_action = this->human_points_vector;
}

void Human::tf_listener()
{
    tf::StampedTransform camTransformInfo;
    tf::TransformListener tfListener;

    while(ros::ok())
    {
        try
        {
            tfListener.waitForTransform("/marker_0",this->camera_name,ros::Time(0),ros::Duration(3.0));
            tfListener.lookupTransform("/marker_0",this->camera_name,ros::Time(0),camTransformInfo);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            continue;
        }

            //计算外参
        Eigen::Matrix<double,3,1> trans(camTransformInfo.getOrigin().getX(),camTransformInfo.getOrigin().getY(),camTransformInfo.getOrigin().getZ());
        this->trans = trans;

        //四元数
        double X = camTransformInfo.getRotation().getX();
        double Y = camTransformInfo.getRotation().getY();
        double Z = camTransformInfo.getRotation().getZ();
        double W = camTransformInfo.getRotation().getW();

        //四元数 --> 旋转矩阵
        Eigen::Quaterniond q(W,X,Y,Z);
        this->cam2world = q.toRotationMatrix();

        ROS_INFO("have got camera info ..");
        break;
    }
}

void Human::change_pose_format(vector<vector<Eigen::Matrix<double,3,1>>>& point_list){
    int human_num = this->human_points_vector.size();
    vector<Eigen::Matrix<double,3,1>> tmp;
    tmp.clear();
    for(int i=0; i<human_num; ++i){
        for(int j=0; j<N; ++j){
            world_xyz(0,0) = this->human_points_vector[i].point[j].x / 100;  //这里记得除以100,因为是在cm单位
            world_xyz(1,0) = this->human_points_vector[i].point[j].y / 100;
            world_xyz(2,0) = this->human_points_vector[i].point[j].z / 100;
            world_xyz = cam2world.cast<double>() * world_xyz + trans;
            tmp.push_back(world_xyz);
        }
        point_list.push_back(tmp);
    }
}

//将人体的姿态信息以tf的方式发布出去
//将按照关节点的顺序进行发布
//这个到时候应该相对marker_0进行发布
void Human::pub_human_info(vector<vector<Eigen::Matrix<double,3,1>>>& point_list){
    tmp_id_action.clear();
    int human_num = this->human_points_vector.size();
    for(int i=0; i<human_num; ++i){
        int human_id = this->human_points_vector[i].human_id;
        int human_action = this->human_points_vector[i].action;
        // cout<<"human_id: "<<human_id<<endl;
        // cout<<"human_action"<<human_action<<endl;
        tmp_id_action.push_back(human_id);
        tmp_id_action.push_back(human_action);
        
        // neck
        // X = this->human_points_vector[i].point[0][0].x;
        // Y = this->human_points_vector[i].point[0][0].y;
        // Z = this->human_points_vector[i].point[0][0].z;
        // current_joint.setOrigin(tf::Vector3(X, Y, Z));
        // current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        // joint_name = to_string(human_id) + "_neck";
        // this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "base", joint_name));

        // root
        X = point_list[i][2](0,0);
        Y = point_list[i][2](1,0);
        Z = point_list[i][2](2,0);
        current_joint.setOrigin(tf::Vector3(X, Y, Z));
        current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        joint_name = to_string(human_id) + "_root";
        this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "marker_0", joint_name)); 
        
        // // lshoulder
        // X = this->human_points_vector[i].point[0][3].x;
        // Y = this->human_points_vector[i].point[0][3].y;
        // Z = this->human_points_vector[i].point[0][3].z;
        // current_joint.setOrigin(tf::Vector3(X, Y, Z));
        // current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        // joint_name = to_string(human_id) + "_lshoulder";
        // this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "base", joint_name));
       
        // lelbow
        X = point_list[i][4](0,0);
        Y = point_list[i][4](1,0);
        Z = point_list[i][4](2,0);
        // point_1 = Eigen::Vector3f(X,Y,Z);  //elbow的原点 --> marker_0
        current_joint.setOrigin(tf::Vector3(X, Y, Z));
        current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        joint_name = to_string(human_id) + "_lelbow";
        this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "marker_0", joint_name));
        
        // lwrist
        X = point_list[i][5](0,0);
        Y = point_list[i][5](1,0);
        Z = point_list[i][5](2,0);
        // point_2 = Eigen::Vector3f(X,Y,Z);
        current_joint.setOrigin(tf::Vector3(X, Y, Z));
        current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        joint_name = to_string(human_id) + "_lwrist";
        this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "marker_0", joint_name));
        
        // // rshoulder
        // X = this->human_points_vector[i].point[0][9].x;
        // Y = this->human_points_vector[i].point[0][9].y;
        // Z = this->human_points_vector[i].point[0][9].z;
        // current_joint.setOrigin(tf::Vector3(X, Y, Z));
        // current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        // joint_name = to_string(human_id) + "_rshoulder";
        // this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "base", joint_name));
       
        // relbow
        X = point_list[i][10](0,0);
        Y = point_list[i][10](1,0);
        Z = point_list[i][10](2,0);
        current_joint.setOrigin(tf::Vector3(X, Y, Z));
        point_1 = Eigen::Vector3f(X,Y,Z);
        current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        joint_name = to_string(human_id) + "_relbow";
        this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "marker_0", joint_name));
       
        // rwrist
        X = point_list[i][11](0,0);
        Y = point_list[i][11](1,0);
        Z = point_list[i][11](2,0);
        point_2 = Eigen::Vector3f(X,Y,Z);
        current_joint.setOrigin(tf::Vector3(X, Y, Z));
        current_joint.setRotation(tf::Quaternion(0.0,0.0,0.0));
        joint_name = to_string(human_id) + "_rwrist";
        this->human_info_br.sendTransform(tf::StampedTransform(current_joint, ros::Time::now(), "marker_0", joint_name));       
    
        // if(human_id == 0){
        //     this->cal_cross_point();
        // }
    
    }
    
    human_id_with_action.data = tmp_id_action;
    this->human_idAction_pub.publish(human_id_with_action);  //发布人的id和相应的动作信息
}

void Human::cal_cross_point(){
    //point_1: lelbow point_2:lwrist
    //设置两个点,并且判断这个手腕的高度是在这个Z的上方的,并且手肘的高度大于手腕的高度

    if(point_1[2] > z_thres && point_2[2] > z_thres && point_1[2] > point_2[2]){
        plane = Eigen::Vector4f(0,0,1,0.2);  //公式：(ax + by + cz + d) = 0
	
        if(this->intersectionLinePlane(point_1, point_2, plane, this->cross_point)){
	    // cout<<cross_point[0]<<endl;
            tf::Transform task_point_cap;
            task_point_cap.setOrigin(tf::Vector3(cross_point[0], cross_point[1], cross_point[2] + 0.15)); //x,y,x --> base
            task_point_cap.setRotation(tf::Quaternion(0.0,0.0,0.0));  //这里的点注意一下
            this->human_info_br.sendTransform(tf::StampedTransform(task_point_cap, ros::Time::now(), "marker_0", "task_point"));
        }
    } 
}

bool Human::intersectionLinePlane(Eigen::Vector3f &p1, Eigen::Vector3f p2, const Eigen::Vector4f &plane, Eigen::Vector3f &crossP){
    //法向量的分子
    auto P1D = plane[0] * p1[0] + plane[1] * p1[1] + plane[2] * p1[2];
    P1D = std::abs(P1D);

    const Eigen::Vector3f P1P2 = p2 - p1;  //指向的wrist
    //法向量的分母
    auto P1D2 = plane[0] * P1P2[0] + plane[1] * P1P2[1] + plane[2] * P1P2[2];
    P1D2 = abs(P1D2);
    if(P1D2 < FLT_EPSILON)
        return false;

    auto m = P1D / P1D2;
    crossP = p1 + m * P1P2;
    
    return true;
}

//把所有的点保存在一个列表
void LinePub::store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList){
    this->line.points.clear();
    for(int i=0; i<pointList.size(); ++i){
        for(int j=0; j<pairNum; ++j){
            geometry_msgs::Point p1, p2;
            int k1 = bodyEages[j][0];
            int k2 = bodyEages[j][1];

            //点
            p1.x = pointList[i][k1](0,0);
            p1.y = pointList[i][k1](1,0);
            p1.z = pointList[i][k1](2,0);
            this->line.points.push_back(p1);

            p2.x = pointList[i][k2](0,0);         
            p2.y = pointList[i][k2](1,0);
            p2.z = pointList[i][k2](2,0);
            this->line.points.push_back(p2);            
        }
    }
}

void PointPub::store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList){
    this->points.points.clear();
    for(int i=0; i<pointList.size(); ++i){
        for(int j=0; j<15; ++j){
            geometry_msgs::Point p1;
            p1.x = pointList[i][j](0,0);
            p1.y = pointList[i][j](1,0);
            p1.z = pointList[i][j](2,0);
            this->points.points.push_back(p1);
        }
    }
}


