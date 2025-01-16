# 一个简单的2Dslam的里程计

## 功能使用
### SLAM建图并生成栅格地图
1. 启动仿真节点和建图节点
```bash
roslaunch robot_launch launch_simulation.launch
roslaunch robot_launch launch_mapping.launch 
```
2. 使用joy_stick控制机器人跑图进行slam
3. 保存点云地图和栅格地图
```bash
rosservice call /map_generator/save_cloud "data: 'your_dir/map.pcd'"

rosservice call /map_generator/save_grid "save_dir: 'your_dir'
resolution: 0.02
occ_prob: 0.8
free_prob: 0.6"
```

### 实时在线重定位
1. 启动仿真节点和里程计节点
```bash
roslaunch robot_launch launch_simulation.launch
roslaunch robot_launch launch_localizer.launch
```
2. 使用joy_stick控制机器人移动到任意位置
3. 在rviz中使用init pose或者调用localizer/relocalize 设置初始位置
```bash
rosservice call /localizer/relocalize "{pcd_path: 'your_pcd_path', reload: false, x: 0.0, y: 0.0, yaw: 0.0}"
```

