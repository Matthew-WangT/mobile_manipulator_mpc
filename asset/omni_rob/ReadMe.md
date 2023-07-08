# Notes
## 1 运行

```
roslaunch omni_rob omni_ur5_gazebo.launch
```

## 2.文件说明

```
-launch
--omni_ur5_gazebo.launch: 移动机器人(MR+ur5)的launch文件
--omni_z1_gazebo.launch: 移动机器人(z1+ur5)的launch文件
-scripts(python3环境)
--base_controller.py: 虚拟的移动机器人(MR)控制器,通过直接调用gazebo的服务来设定MR的位姿.
--camera_trans_pub.py: 相机的位姿态发布器,给fiesta用.
--fake_odom.py:发布获取自gazebo信息的真实odom,而非基于轮子里程计.
--robo_teleop.py:使用键盘控制机器人.
```
