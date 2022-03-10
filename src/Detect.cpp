#include<new_hri/Detect.h>

void Object::load_camera_param(){
    cv::FileStorage fs(this->_camera_param_path, cv::FileStorage::READ);  //相机内参
    if(!fs.isOpened()){
        ROS_INFO("cannot open the configuration file");
        exit(1);
    }
    fs["camera_matrix"] >> this->_camera_matrix;
    fs.release();

    cv::Mat distCoeffs_origin = (Mat_<double>(1,5) << 0,0,0,0,0);
    this->_camera_dist = distCoeffs_origin.clone();
    cout<<"读取内参文件成功 .."<<endl;
}

void Object::img_callback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv_ptr->image.copyTo(this->_img);
        // cout<<"get image .."<<endl;
    }
    catch(cv_bridge::Exception& ex){
        // ROS_INFO("cv_bridge exception: %s", ex.what());
        return;
    }

}

void Object::sendMarkerTF(cv::Vec3d& marker_trans,cv::Vec3d& marker_rot,int &id){
    //这里应该是固定好机械臂的夹取的旋转四元数信息，配合空间位置
    cv::Mat rot_(3, 3, CV_64FC1);
    static tf::TransformBroadcaster marker_position_broadcaster;
    cv::Rodrigues(marker_rot, rot_);   //罗得里格斯公式 旋转矩阵  物块marker --> 相机
    rot_.convertTo(rot_, CV_64FC1);
    cv::Mat rot = rot_;

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
    tf::Vector3 tf_trans(marker_trans(0), marker_trans(1), marker_trans(2));  //cv -> tf　　这个用检测到的空间position
    cout<<marker_trans(0)<<" "<<marker_trans(1)<<" "<<marker_trans(2)<<endl;
    tf::Transform transform(tf_rot,tf_trans);
    

    tf::Transform transform_new;
    auto q = transform.getRotation();
    q.setRPY(0,M_PI,0);  //y轴旋转90
    transform_new.setRotation(q);
    transform_new.setOrigin(tf::Vector3(0.0+_axis_trans_x, 0.0+_axis_trans_y, 0.0+_axis_trans_z));

    ostringstream oss;
    oss << "marker_" << id;
    marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_base_1", oss.str()));
    marker_position_broadcaster.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), oss.str(), "new"));
    ROS_INFO("send %d tf info ..", id);
}

void Object::locate_marker(){
    this->_markerIDs.clear(); //清空一次这个
    this->_markerCorners.clear();
    this->_tvecs.clear();
    this->_rvecs.clear(); 
    // cout<<!this->_img.empty()<<endl;   //这里订阅不到图像诶  原因是：话题错误
    if(!this->_img.empty()){
        cv::aruco::detectMarkers(_img, _dictionary, _markerCorners, _markerIDs);  //检测marker
        int marker_id_size = _markerIDs.size();
        // cout<<marker_id_size<<endl;
        // cout<<"marker的数量: "<<marker_id_size<<endl;
        if(marker_id_size > 0){
            cv::aruco::drawDetectedMarkers(_img, _markerCorners, _markerIDs);
            
            for(int idx = 0; idx < marker_id_size; ++idx){
                int marker_id = _markerIDs[idx];
                cout<<marker_id<<endl;
                if ((marker_id > 0)){
                    cv::aruco::estimatePoseSingleMarkers(_markerCorners, 0.05, _camera_matrix, _camera_dist, _rvecs, _tvecs);
                    cv::aruco::drawAxis(_img, _camera_matrix, _camera_dist, _rvecs[idx], _tvecs[idx], 0.1);
                    this->sendMarkerTF(_tvecs[idx], _rvecs[idx], marker_id);
                }
            }
        }
        char window_name[50];
        sprintf(window_name,"camera_%d",1);
        cv::imshow(window_name,this->_img);
        cv::waitKey(3);
    }

}

void Object::sendMarkerID(){
    _tmp_marker_vec.clear();
    int marker_id_size = _markerIDs.size();
    for (int i=0; i<marker_id_size; ++i){
        int marker_id = _markerIDs[i];
        if (marker_id > 0){
            _tmp_marker_vec.push_back(marker_id);
        }
    }
    _marker_id_list.data = _tmp_marker_vec;
    _marker_id_pub.publish(_marker_id_list);
}

void Object::run(){
    ros::Rate rate(60);

    while(ros::ok()){
        ros::spinOnce(); //得到相机画面
        // cout<<"123"<<endl;
        //1.load marker & send marker tf
        locate_marker();
        //2.send marker id list
        sendMarkerID();

        rate.sleep();
    }
}