2021.12.23
new_hri是之后要用到的控制机器人的部分
目前写了人体姿态的话题接收（用到了三个.msg文件，但是在包外编译的话，程序找不到这个，很奇怪）
       到时候，人体的姿态在python端可以已经转化到世界坐标系中，也可以在c++端进行转化
       爪子的夹取和松开
       机器人的控制（包括机器人的关节角度和角速度的实时读取，对于到时候的物块的tf追踪，向机器人实时发送控制信息等）
但是还有别的东西没有完善．．
包括机器人到时候根据人体的姿态执行任务等的控制策略．．
