#pragma once
#include<iostream>
#include<new_hri/Gripper.h>
#include<new_hri/Robot.h>
using namespace std;

class Modules{
public:
    Modules(){
        robot = new Robot;
        gripper = new Gripper;
    }

    void run();
    void catch_object();
    void go_task_point();

    ~Modules(){
        delete robot;
        delete gripper;
    }

private:
    Robot *robot;
    Gripper *gripper;
};