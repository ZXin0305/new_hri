#include<new_hri/Robot.h>

void Robot::init(){
    //关节名称
	joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    //初始化到初始点
    this->mode = mode1; //初始化模式１
    this->toInitPoint = true;

    this->toCapPoint = false;
    this->toPlacePoint = false;
    this->toObjPoint = false;
    this->obj_flag = 0;
    this->stage = "go";

    this->kD = pow((max_thres * min_thres), 2) / pow((max_thres - min_thres), 2);
}

//我觉得这里可以设置成根据人手腕和机器人末端的距离，来调整机器人的运行速度
void* Robot::goal_rectify_thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;
    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.85;   //工作空间
    double base_radius = 0.2;  //

    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;  //对姿态进行修正

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < 0.15){
                rectified_position.setZ(0.15);
                rect_goal.setOrigin(rectified_position);
                ROS_WARN("goal under the working surface, setting rectified goal\n");
            }

            double desired_distance = rectified_position.length();

            if (desired_distance > work_space_radius)
            {
                double scale = work_space_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal outoff robot work space! setting rectified goal\n");
            }

            desired_distance = rectified_position.length();
            if (desired_distance < base_radius){
                double scale = base_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal too close to base! setting rectified goal\n");
            }
            br.sendTransform(tf::StampedTransform(rect_goal, ros::Time::now(), "base", "rect_goal"));
        }
        catch(tf::TransformException ex){
        }
        rate.sleep();
    }
}

void Robot::jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();  //关节角度
    joint_speed.clear();   //关节速度
    std::vector<std::string> joint_names_recv = joint_state.name;  //关节名称  .name是一个列表
    for(auto it = joint_names.begin(); it !=joint_names.end(); ++it)
    {
        for(auto it_recv = joint_names_recv.begin(); it_recv != joint_names_recv.end(); ++it_recv)
        {
            if (*it_recv == *it)
            {
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);  //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);   //关节角速度
                break;
            }
        }
    }
    JointState_available = true;
    if(_start_step == 0) 
    {
        _start_step = 1;  //第一个采样步不用进行,并且让两个相等。。。
        _last_base_tool_vector = this->listener_robot_point(_joint_name_tool0);
        _base_tool_vector = _last_base_tool_vector;
    }    
}

void Robot::sendCmd(TRAC_IK::TRAC_IK& ik_solver, tf::StampedTransform& tf_point){
    transformTFToKDL(tf_point, desired_pose);

    if(JointState_available){
        for(int i=0; i<JOINT_NUM; ++i){
            q(i) = joint_angles[i];   //q是joint_angles（相当于上一时刻）
            printf("q_current[%d]:%f\n", i, joint_angles[i]);
        }

        if(ik_solver.CartToJnt(q, desired_pose, q_desired))   //返回期望的q_desired，逆运动学求解器
        {
            std::vector<double> speed_vector;  

            for(int j=0; j<JOINT_NUM; ++j){
                double delta = q_desired(j) - q(j);  //误差（q_desired:系统输入，q：上一时刻的“输出”）
                double speed = Kp*delta + Td*joint_speed[j]; //PD控制--比例环节:当前偏差，微分环节：最近偏差

                //速度判断 //这里可以判断根据人体的姿态进行速度的调整
                if(speed > max_speed){
                    speed = max_speed;
                }
                if(speed < -max_speed){
                    speed = -max_speed;
                }
                speed_vector.push_back(speed);
                printf("speed[%d]:%f\n",j,speed);   //速度控制
            }

            // judge speed coreponse to the distance
            this->adjust_speed(speed_vector);

            char command[100];  //command
            sprintf(command, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", 
                            speed_vector[0], speed_vector[1], speed_vector[2], 
                            speed_vector[3], speed_vector[4], speed_vector[5]);
            // ROS_INFO("ur script send: %s", command);
            command_msg.data = command;
            ur_command_publisher.publish(command_msg);

        }
    }
}

double Robot::cal_distance(tf::StampedTransform& tf_point, string& robot_joint_name){
    tf::Vector3 base_point_vector;  //机器人base到需要测量的点的距离向量
    tf::Vector3 base_tool_vector;
    base_point_vector = tf_point.getOrigin();
    base_tool_vector = this->listener_robot_point(robot_joint_name);

    //矢量相加
    //这里就不是只有human的joint可以用了
    tf::Vector3 robot_point_vec = base_point_vector - base_tool_vector;
    double distance = robot_point_vec.length();
    return distance;
}


void Robot::sort_distance()
{
    this->distance_vec.clear();
    vector<int> distance_idx(all_human_joint_tf.size()); //初始化距离的索引列表
    for(int i=0; i<this->all_human_joint_tf.size(); ++i){
        double dis = this->cal_distance(this->all_human_joint_tf[i], _joint_name_tool0);
        distance_vec.push_back(dis);
    }
    sort(distance_idx.begin(), distance_idx.end(),
        [&](const int& a, const int& b) {
            return (distance_vec[a] < distance_vec[b]);
        }
    );     //　索引排序好了   
    sort(distance_vec.begin(), distance_vec.end());  //默认：从小到大
    
    if(distance_vec[0] <= 0.2)
        this->_robot_is_cooperate = true;
    else
        this->_robot_is_cooperate = false;
    
    this->_closest_dis = distance_vec[0];
    this->_closest_idx = distance_idx[0];
}

void Robot::adjust_speed(vector<double>& speed_vector){

    //当前中所有人的点都给计算出来了距离
    if(this->all_human_joint_tf.size() > 0)
    {
        this->sort_distance();
        double closest_dis = this->_closest_dis; //最近的点
        int closest_idx = this->_closest_idx; //最近点的索引
        double speed_factor = 1;

        // *************************************************************************************
        _base_tool_vector = this->listener_robot_point(_joint_name_tool0);
        tf::Vector3 closest_human_joint = all_human_joint_tf[closest_idx].getOrigin();
        //1.先求出机器人的运动趋势的向量  并求出对应的模长
        tf::Vector3 robot_trend = _base_tool_vector - _last_base_tool_vector;
        double length_robot_trend = robot_trend.length();
        ROS_INFO("length_robot_trend    ---->      %f", length_robot_trend);
        //2.求出机器人采样帧到最近点的距离  并求出对应的模长
        tf::Vector3 robot2closest = closest_human_joint - _last_base_tool_vector;
        double length_robot2closest = robot2closest.length();
        ROS_INFO("length_robot2closest    ---->      %f", length_robot2closest);

        //3.求出二者的点乘  tf的点乘得到的还是一个tf向量
        tf::Vector3 point_mul = (robot_trend * robot2closest);
        double tmp = point_mul[0] + point_mul[1] + point_mul[2];
        double theta_ = acos((double)(tmp / length_robot2closest));
        ROS_INFO("theta    ---->     %f", theta_);

        if(theta_ <= _max_trend_thres) //小于的话，就进行速度调整
        {
            // ofstream theta_data(_data_path, ios::app);
            // theta_data << theta_;
            // theta_data.close();
            if(closest_dis <= max_thres && closest_dis > min_thres){
                double tmp = pow((1/closest_dis - 1/max_thres), 2);
                speed_factor = exp((-1) * (kD * tmp));
                for(int i=0; i<JOINT_NUM; ++i){
                    speed_vector[i] *= speed_factor;
                }
            }
            if(closest_dis >= 0 && closest_dis <= min_thres){
                speed_factor = 0;
                for(int i=0; i<JOINT_NUM; ++i){
                    speed_vector[i] *= speed_factor;
                }            
            }
        }

        

        //*******************************************************************************
        //最近的点
        // double closest_dis = distance_vec[0]; //最近的点
        // double speed_factor = 1;
        // if(closest_dis > max_thres)
        //     return;
        // if(closest_dis <= max_thres && closest_dis > min_thres){
        //     double tmp = pow((1/closest_dis - 1/max_thres), 2);
        //     speed_factor = exp((-1) * (kD * tmp));
        //     for(int i=0; i<JOINT_NUM; ++i){
        //         speed_vector[i] *= speed_factor;
        //     }
        // }
        // if(closest_dis >= 0 && closest_dis <= min_thres){
        //     speed_factor = 0;
        //     for(int i=0; i<JOINT_NUM; ++i){
        //         speed_vector[i] *= speed_factor;
        //     }
        // }
    
    }
    //就算没有人的话，也需要对这个对这个进行采样
    if(this->_update_flag == this->_update_rate)
    {
        _base_tool_vector = this->listener_robot_point(_joint_name_tool0);
        _last_base_tool_vector = _base_tool_vector;  //更新
        this->_update_flag = 0;
    }
    else
        this->_update_flag++;

    ofstream theta_data(_data_path, ios::app);
    theta_data << _update_flag;
    theta_data.close();
    
    return;
}

tf::Vector3 Robot::listener_robot_point(string& robot_joint_name)
{
    tf::Vector3 tf_vec;
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform("base", robot_joint_name, ros::Time(0), base_tool_dis); // base --> tool0
            tf_vec = base_tool_dis.getOrigin();
            break;
        }
        catch(tf::TransformException &ex){
            // ROS_INFO("continue to track base & tool0 ..");
            continue;
        }
    }
    return tf_vec;
}

// double Robot::cal_distance(tf::StampedTransform& tf_point){
//     tf::Vector3 base_point_vector;  //机器人base到需要测量的点的距离向量
//     tf::Vector3 base_tool_vector;
//     base_point_vector = tf_point.getOrigin();
//     while(ros::ok()){
//         try{
//             this->tf_listener.lookupTransform("base", "tool0", ros::Time(0), base_tool_dis); // base --> tool0
//             base_tool_vector = base_tool_dis.getOrigin();
//             break;
//         }
//         catch(tf::TransformException &ex){
//             // ROS_INFO("continue to track base & tool0 ..");
//             continue;
//         }
//     }
//     //矢量相加
//     //这里就不是只有human的joint可以用了
//     tf::Vector3 robot_point_vec = base_point_vector - base_tool_vector;
//     double distance = robot_point_vec.length();
//     return distance;
// }


//这里是监听抓取物体的任务目标点
void Robot::listener_object(){

    while(ros::ok()){
        this->correct_target.clear();
        try{
            //这里还有其他的点
            this->tf_listener.lookupTransform("base", "obj0_correct", ros::Time(0), transform_correct);
            this->tf_listener.lookupTransform("base", "obj0_target", ros::Time(0), transform_target);
            this->correct_target.push_back(transform_correct);
            this->correct_target.push_back(transform_target);

            // this->tf_listener.lookupTransform("base", "obj1_correct", ros::Time(0), transform_correct);
            // this->tf_listener.lookupTransform("base", "obj1_target", ros::Time(0), transform_target);
            // this->correct_target.push_back(transform_correct);
            // this->correct_target.push_back(transform_target);

            // this->tf_listener.lookupTransform("base", "obj2_correct", ros::Time(0), transform_correct);
            // this->tf_listener.lookupTransform("base", "obj2_target", ros::Time(0), transform_target);
            // this->correct_target.push_back(transform_correct);
            // this->correct_target.push_back(transform_target);

            // *************************************************************************************************** //
            // this->tf_listener.lookupTransform("base", "obj3_correct", ros::Time(0), transform_correct);
            // this->tf_listener.lookupTransform("base", "obj3_target", ros::Time(0), transform_target);
            // this->correct_target.push_back(transform_correct);
            // this->correct_target.push_back(transform_target);
            // this->tf_listener.lookupTransform("base", "obj4_correct", ros::Time(0), transform_correct);
            // this->tf_listener.lookupTransform("base", "obj4_target", ros::Time(0), transform_target);
            // this->correct_target.push_back(transform_correct);
            // this->correct_target.push_back(transform_target);
            // this->tf_listener.lookupTransform("base", "obj5_correct", ros::Time(0), transform_correct);
            // this->tf_listener.lookupTransform("base", "obj5_target", ros::Time(0), transform_target);
            // this->correct_target.push_back(transform_correct);
            // this->correct_target.push_back(transform_target);

            obj_num = this->correct_target.size();
            
            break;
        }
        catch(tf::TransformException &ex){
            continue;
        }
    }
}

//这里是监听初始点、抓取点以及放置点
void Robot::listener_point(){
    while(ros::ok()){
        this->cap_init.clear();
        try{
            this->tf_listener.lookupTransform("base", "cap_point", ros::Time(0), cap_point);  //0
            this->cap_init.push_back(cap_point);
            this->tf_listener.lookupTransform("base", "init_point", ros::Time(0), init_point); //1
            this->cap_init.push_back(init_point);
            this->tf_listener.lookupTransform("base", "place_point", ros::Time(0), place_point); //2
            this->cap_init.push_back(place_point);            
            // ROS_INFO("得到初始点和过渡点以及放置点　..");
            break;
        }
        catch(tf::TransformException &ex){
            continue;
        }
    }
}

//小于1cm说明达到该位置
bool Robot::judge(tf::StampedTransform& tf_point){
    double length = this->cal_distance(tf_point, _joint_name_tool0);
    if(length <= 0.01)
        return true;
    return false;
}

bool Robot::isToPlace()
{
    bool is_toPlace = this->toPlace;
    return is_toPlace;
}

//监听得到当前的人员的id 以及相应的动作
void Robot::humanIdActionCallback(const std_msgs::Int32MultiArray& id_action_array){
    tmp_id_action.clear();
    tmp_id_action = id_action_array.data;
    human_num = tmp_id_action.size() / 2;  //因为在存储的时候就是两个一起的
 
    if(human_num == 0){
        ROS_INFO("here is no people ..");
        return;
    }

    //这样会把所有人的关节点的信息都放在了一个vector中
    //先将关节点数设为5
    string joint_name;

    tf::StampedTransform tmp;
    while(ros::ok()){
        all_human_joint_tf.clear();
        try{
            for(int i=0; i<human_num; ++i){
                int human_id = tmp_id_action[i * 2];  //按照顺序得到对应的human的id
                // ROS_INFO("current human id: %d", human_id);
                joint_name = to_string(human_id) + "_root";
                this->tf_listener.lookupTransform("base", joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                joint_name = to_string(human_id) + "_lelbow";
                this->tf_listener.lookupTransform("base", joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                joint_name = to_string(human_id) + "_lwrist";
                this->tf_listener.lookupTransform("base", joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                joint_name = to_string(human_id) + "_relbow";
                this->tf_listener.lookupTransform("base", joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                joint_name = to_string(human_id) + "_rwrist";
                this->tf_listener.lookupTransform("base", joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);  
                // ROS_INFO("get human %d joints ..", human_id);          
            }
            break;
        }
        catch(tf::TransformException &ex){
            ROS_INFO("wait for human info ..");
            continue;
        }
    }
}

//　就使用固定点吧
// void Robot::objectIdCallback(const std_msgs::Int32MultiArray& object_id_array)
// {
//     _object_id.clear();
//     _object_id = object_id_array.data;
//     task_num = _object_id.size();
//     if(task_num == 0)
//     {
//         ROS_INFO("here is no object .. ");
//         return;
//     }
//     sort(_object_id.begin(), _object_id.end());   //从小达到的排序
//     string task_point_name;
//     tf::StampedTransform tmp;
//     while(ros::ok())
//     {
//         all_object_tf.clear();
//         try
//         {         
//         }
//         catch(tf::TransformException &ex)
//         {
//         }      
//     }
// }

void Robot::track(){
    ros::spinOnce();
    ros::Rate rate(60);
    //　模式是到达初始点，并且toInitPoint为真
    if(this->mode == mode1 && this->toInitPoint){
        this->reach_point("cap_init[1]");

        if(!this->toCapPoint && this->stage == "go")
        {
            this->toInitPoint = false;
            this->toCapPoint = true;
            this->mode = mode2;
        }
        if(!this->toPlacePoint && this->stage == "back")
        {
            this->toInitPoint = false;
            this->toPlacePoint = true;
            this->mode = mode3;
        }
    }
    else if(this->mode == mode2 && this->toCapPoint)
    {
        this->reach_point("cap_init[0]");

        if(!this->toObjPoint && this->stage == "go")
        {
            this->toCapPoint = false;
            this->toObjPoint = true;
            this->mode = mode4;
        }
        if(!this->toInitPoint && this->stage == "back")
        {
            this->toCapPoint = false;
            this->toInitPoint = true;
            this->mode = mode1;
        }
    }
    else if(this->mode == mode3 && this->toPlacePoint)
    {
        this->reach_point("cap_init[2]");

        //mode3 \ mode4不考虑stage是因为这两个模式都只关联到一个点
        if(!this->toInitPoint)
        {
            this->toPlacePoint = false;
            this->toInitPoint = true;
            this->mode = mode1;
            this->stage = "go";
        }
    }
    else if(this->mode == mode4 && this->toObjPoint)
    {
        //矫正点
        this->reach_point("obj_correct");

        //抓取点
        this->reach_point("obj_capture");

        obj_flag++; //抓取过前面的物块之后，累加，抓取后面的点
        if(obj_flag == (this->correct_target.size() / 2 - 1))
            obj_flag = 0;
        if(!this->toCapPoint)
        {
            this->toObjPoint = false;
            this->toCapPoint = true;
            this->mode = mode2;
            this->stage = "back";
        }
    } 
}

void Robot::test(){
    // ros::spinOnce();
    // this->distance_vec.clear();
    // for(int i=0; i<this->all_human_joint_tf.size(); ++i){
    //     double dis = all_human_joint_tf[i].getOrigin().length();
    //     distance_vec.push_back(dis);
    //     cout<<dis<<endl;
    // }

    // if(distance_vec.size() > 0){
    //     sort(distance_vec.begin(), distance_vec.end());
    //     cout<<"最近距离为: "<<distance_vec[0]<<endl;
    // }
    // else{
    //     cout<<"no human"<<endl;
    // }
    // this->listener_point();
    // this->listen_action();
    // reach_point("cap_init[1]");
    // reach_point("cap_init[0]");
    // reach_point("cap_init[2]");
    // this->listen_reach_task();
    // int human_action =  this->listen_action();
    // if(human_action == 2 && _test_num == 0)
    // {
    //     reach_point("cap_init[1]");
    //     _test_num++;
    // }
    // else if(human_action == 2 && _test_num == 1)
    // {
    //     reach_point("cap_init[0]");
    //     _test_num++;        
    // }
    // else if(human_action == 2 && _test_num == 2)
    // {
    //     reach_point("cap_init[2]");
    //     _test_num = 0;
    // }

    reach_point("cap_init[1]");  //home
    sleep(2);
    reach_point("cap_init[0]");  //cap
    sleep(2);
    // reach_point("cap_init[1]");  //home
    // sleep(1);
    reach_point("cap_init[2]");  //place
    sleep(2);

    // reach_point("obj_correct");
    // reach_point("obj_capture");
    // sleep(2);
    // reach_point("obj_correct");
    // sleep(0.1);
    // obj_flag++;
    // if(obj_flag == 2)
    // {
    //     obj_flag = 0;
    // }
    // reach_point("cap_init[0]");
    // sleep(0.1);
    // reach_point("cap_init[1]");
    // sleep(0.1);
    // reach_point("cap_init[2]");
}


/*
human action:
    0:standby
    1:walk 
    2:wave arm
    3:open arm
    4:raise hand
    -1:no action -->　继续等待
*/
bool Robot::listen_reach_task(){
    ros::Rate rate(60);
    float time_decaly = 0;
    bool get_task = false;
    TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);

    while(ros::ok()){
        if(time_decaly >= delay_time)
        {
            ROS_INFO("failed to catch the task point, carry other task ..");
            break;
        }
        clock_t st = clock();
        
        try{
            this->tf_listener.waitForTransform("base","task_point",ros::Time(0),ros::Duration(3.0));
            this->tf_listener.lookupTransform("base", "task_point",ros::Time(0),task_point);
            get_task = true;
            ROS_INFO("have get the task point");
            break;
        }
        catch(tf::TransformException &ex){
            clock_t et = clock();
            ROS_INFO("please point out task point ..");
            time_decaly += (et - st) / CLOCKS_PER_SEC;
            continue;
        }
    }

    // 到达物件固定点
    if(get_task){
        while(ros::ok()){
            ros::spinOnce();
            this->sendCmd(ik_solver, task_point);
            if(this->judge(task_point))
                break;
            rate.sleep();
        }
        return true;
    }

    return false;
    
}

int Robot::listen_action(){
    int human_action = -2;
    ros::Rate rate(60);
    while(ros::ok()){
        ros::spinOnce();
        int human_num = this->tmp_id_action.size() / 2;

        for(int i=0; i<human_num; ++i){
            int human_id = this->tmp_id_action[i*2];
            if(human_id == 0)  //这里就设定ID为0的人为操作者
            {
                human_action = (int)this->tmp_id_action[i*2+1];
                break;
            }
            else
                human_action = -2;
        }  

        if(human_action != -2)
        {
            ROS_INFO("human_action --> %d", human_action);
            break;
        }

        rate.sleep();
    }

    return human_action;

}

void Robot::reach_point(string case_){
    TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    ros::Rate rate(60);
    tf::StampedTransform tf_point;
    int flag = 0;
    while(ros::ok()){
        ros::spinOnce();

        if(case_ == "cap_init[1]"){
            this->listener_point();
            tf_point = this->cap_init[1];  //home           
        }
        else if(case_ == "cap_init[0]"){
            this->listener_point();
            tf_point = this->cap_init[0];  //过渡点            
        }
        else if(case_ == "cap_init[2]"){
            this->listener_point();
            tf_point = this->cap_init[2];  //放置点            
        }
        else if(case_ == "obj_correct"){
            ROS_INFO("here --------->    ");
            this->listener_object();
            tf_point = this->correct_target[obj_flag * 2];            
        }
        else if(case_ == "obj_capture"){
            this->listener_object();
            tf_point = this->correct_target[obj_flag * 2 + 1];            
        }
        
        this->sendCmd(ik_solver, tf_point);
  
        if(this->judge(tf_point)){
            break;
        }
        rate.sleep();
    }
}

int Robot::get_correct_target_size()
{
    return correct_target.size();
}
