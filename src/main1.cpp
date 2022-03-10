#include<new_hri/common.h>

void Modules::catch_object()
{
    gripper->loose();
    sleep(0.1);
    robot->reach_point("cap_init[1]");   //home
    robot->reach_point("cap_init[0]");   //过渡点
    robot->reach_point("obj_correct");   //物件的抓取过渡点
    robot->reach_point("obj_capture");   //达到抓取点
    sleep(0.2);
    // gripper->clamp();
    sleep(0.2);
    robot->reach_point("cap_init[0]");   //过渡点 
    robot->reach_point("cap_init[1]");   //home 
    robot->reach_point("cap_init[2]");   //物件放置点
    sleep(2);
    // gripper->loose();
    robot->obj_flag++;  //物件的个数
    int correct_target_size = robot->get_correct_target_size();
    if(robot->obj_flag == (correct_target_size / 2))
        robot->obj_flag = 0;    
}

void Modules::go_task_point()
{
    bool reach_task = robot->listen_reach_task();
    sleep(1);
    if(reach_task)
    {
        gripper->clamp();        
    }    
}

void Modules::run(){

    // 初始化　...
    gripper->loose();  //回到初始位置，并张开爪子
    sleep(1);
    robot->reach_point("cap_init[1]");

    ros::Rate rate(60);

    // // 实验1
    // while(ros::ok()){
    //     robot->track();  //包括spinonce和rate.sleep()
    //     if(robot->mode == mode2 && robot->stage == "back")
    //     {
    //         sleep(0.5);
    //         gripper->clamp();
    //     }
    //     else if(robot->mode == mode1 && robot->stage == "go")
    //     {
    //         sleep(0.5);
    //         gripper->loose();
    //         sleep(2);  //如果使用指定位置的话，这里要暂停一段时间，等机械臂这边监听到指定的位置
    //     }
        //实验2的部分
        // if(robot->mode == mode1 && robot->stage == "go")
        // {   
        //     if(robot->obj_flag != 0)
        //     {
        //         auto get_task = robot->listen_reach_task();  //这个是到达那个机械臂的固定物件点
        //         sleep(0.5);
        //         if(get_task)
        //             gripper->clamp();    //如果监听到了这个目标点，才会到达进行夹取
        //         bool have_done = false;
        //         while(ros::ok()){
        //             int action = robot->listen_action();
        //             if(action != 2 && action != 3)
        //                 continue;
        //             else if (action == 2){
        //                 sleep(0.5);
        //                 if (get_task)
        //                     gripper->loose();
        //                 break;
        //             }
        //             else if (action == 3 && !have_done)
        //             {
        //                 sleep(0.5);
        //                 gripper->loose();
        //                 robot->reach_point("cap_init[1]");
        //                 have_done = true;
        //             }  
        //         }
        //     }
        // }
    // }

    //实验
    ROS_INFO("robot is ready ..");

    while(ros::ok())
    {
        ros::spinOnce();
        int human_action = -1;
        if(robot->all_human_joint_tf.size() > 0)
            robot->sort_distance();   //通过判断最近的距离，然后看机器人是不是处于协作状态或者运行中,是->就不监听动作了
        if(!robot->_robot_is_cooperate)
        {
            human_action = robot->listen_action();
            switch (human_action)
            {
                case -1:
                    break;
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    this->catch_object();  //抓取物件，并放到人的面前
                    sleep(3);
                    break;
                case 3:
                    gripper->loose();
                    robot->reach_point("cap_init[1]");  //回到初始点
                    break;
                case 4:
                    this->go_task_point();  //去到协助点并抓取
                    break;
                default:
                    break;
            }
        }
        robot->_robot_is_cooperate = true;
        cout<<human_action<<endl;

        rate.sleep();

    }
    

    //这个主要是为了测试能否接收到human那边发来的信息
    // while(ros::ok()){
    //     robot->test();
    //     rate.sleep();
    // }

    // 测试爪子
    // while(ros::ok())
    // {
    //     gripper->clamp();
    //     sleep(1);
    //     gripper->loose();
    //     rate.sleep();
    // }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_thread");
    ros::NodeHandle nh;
    Modules modules;
    modules.run();
    return 0;

}
