#include<new_hri/Points.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_points");
    ros::NodeHandle nh;
    ros::Rate rate(60);
    Points points;
    while(ros::ok()){
        points.pub_points();
        rate.sleep();
    }
}