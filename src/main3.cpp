#include<new_hri/Detect.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "detect_pub_markers");
    ros::NodeHandle nh;
    string camera_topic = "/kinect2_1/hd/image_color";
    string camera_param_path = "/home/zx/zx_ws/src/new_hri/cam_data/cam.yml";  //这个是相机的内参数
    Object marker_detector(camera_topic, camera_param_path);
    marker_detector.run();
}