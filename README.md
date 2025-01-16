# 一个简单的2Dslam的里程计
没什么花里胡哨，主打一个简单，且支持超大规模地图

## 编译运行

- 编译
```bash
catkin_make
```
- 启动
```bash
roslaunch voxel_mapping voxel_mapping.launch 
```
- 播包
```bash
rosbag play xxx.bag
```

## 数据集(webot的仿真环境)
```text
链接: https://pan.baidu.com/s/1DDiUS1tVqJWGoJzLfjS8JA?pwd=4wmp 提取码: 4wmp 
--来自百度网盘超级会员v8的分享
```
## 其他分支
- main: 基于voxel实现的最基础的2dslam；
- fuse_imu: 基于误差迭代卡尔曼，简单融合imu数据，输出带速度的Odomtery；
- localizer: 提供了基础的地图保存功能(点云地图和占用栅格地图)，同时基于nanoflan实现了一个plicp算法用于支持**在线重定位**功能；
