# ILIVE


配置
```
git clone https://github.com/SLAMLab-CUHKSZ/ILIVE.git
cd ILIVE
catkin_make
source ./devel/setup.bash
```




---------manual log--------
2024/4/20

修改by lhy
1. 把topic /aft_mapped_to_init 加入pose的协防差信息(6×6)

<nav_msgs::Odometry> 
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose  --- 这个库里面的协防差是36维度(6*6,针对pose的)
geometry_msgs/TwistWithCovariance twist

2. 发布一个专门的16*16的协防差. topic name: covariance16

