#include<iostream>
#include<vector>
#include<algorithm>
#include<cmath>
#include<tf/tf.h>
using namespace std;

int main(int argc, char ** argv)
{
    // vector<int> tmp{3,2,1,4,2};
    // vector<int> idx(5);
    // for(int i=0; i<idx.size(); ++i) idx[i] = i;
    // // sort(idx.begin(), idx.end(), [&tmp](int i1, int i2) -> bool({return tmp[i1] < tmp[i2]; }));

    // sort(idx.begin(), idx.end(),
    //     [&](const int& a, const int& b) {
    //         return (tmp[a] < tmp[b]);
    //     }
    // );
    // sort(tmp.begin(), tmp.end());
    // for(int i=0; i<tmp.size(); ++i)
    // {
    //     // cout<<idx[i]<<" ";
    //     cout<<tmp[i]<<" ";
    // }
    // cout<<endl;
    
    // double theta = acos(0);
    // cout<<theta<<endl;

    // //1.先求出机器人的运动趋势的向量  并求出对应的模长
    // tf::Vector3 robot_trend(0,0,1);
    // double length_robot_trend = robot_trend.length();
    // //2.求出机器人采样帧到最近点的距离  并求出对应的模长
    // tf::Vector3 robot2closest(1,0,0);
    // double length_robot2closest = robot2closest.length();

    // //3.求出二者的点乘  tf的点乘得到的还是一个tf向量
    // tf::Vector3 point_mul = (robot_trend * robot2closest);
    // double tmp = point_mul[0] + point_mul[1] + point_mul[2];
    // double theta_ = acos((double)(tmp / length_robot2closest));
    // cout<<theta_<<endl;

    cout<<M_PI / 2<<endl;
    
    return 0;

}