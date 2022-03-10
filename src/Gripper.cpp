#include<new_hri/Gripper.h>

void Gripper::clamp()
{
    srv_move.request.position = 10;
    srv_move.request.speed = 100;
    srv_move.request.acceleration = 50;
    srv_move.request.torque = 1;
    srv_move.request.tolerance = 100;
    srv_move.request.waitFlag = true;
    std::cout<<"进行抓取........"<<std::endl;
    move_to_client.call(srv_move);

}

void Gripper::loose()
{
    srv_move.request.position = 80;
	srv_move.request.speed = 100;
	srv_move.request.acceleration = 50;
	srv_move.request.torque = 1;
	srv_move.request.tolerance = 100;
	srv_move.request.waitFlag = true;
    std::cout<<"放下.........."<<std::endl;
	move_to_client.call(srv_move);
}