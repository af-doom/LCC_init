# #!/bin/bash
# command1="source /home/wyw/ROS1_PROJECT/BD/lidar_imu_init/devel/setup.bash;roslaunch lidar_imu_init velodyne.launch;bash"
# command2="sleep 1s;source /home/wyw/ROS1_PROJECT/vins-calib/devel/setup.bash;roslaunch vins_estimator realsense_color.launch;bash"
# command3="sleep 1s;source /home/wyw/ROS1_PROJECT/vins-calib/devel/setup.bash;roslaunch vins_estimator vins_rviz.launch;bash"
# command4="sleep 1s;rosbag play /home/wyw/ROS1_PROJECT/BD/calib_data/wyw/2022-11-25-15-12-16.bag;bash"
# ## Modify terminator's config
# sed -i.bak "s#COMMAND1#$command1#; s#COMMAND2#$command2#; s#COMMAND3#$command3#; s#COMMAND4#$command4#;" ~/.config/terminator/config

# ## Launch a terminator instance using the new layout
# terminator -l default

# ## Return the original config file
# mv ~/.config/terminator/config.bak ~/.config/terminator/config
# 打印下测试信息
#!/bin/bash
echo "test for the BD_ICL..."

# 第一步：启动ROS节点管理器
gnome-terminal -t "start_bd_IL" -x bash -c "source /home/wyw/ROS1_PROJECT/BD/lidar_imu_init/devel/setup.bash;roslaunch lidar_imu_init velodyne.launch;bash"
# 睡眠1s
sleep 1s

# 第二步：启动小乌龟节点
gnome-terminal -t "start_bd_IC" -x bash -c "source /home/wyw/ROS1_PROJECT/vins-calib/devel/setup.bash;roslaunch vins_estimator realsense_color.launch;bash"
# 睡眠1s
sleep 1s

# 第三布：启动键盘控制节点
gnome-terminal -t "start_VISUAL" -x bash -c "source /home/wyw/ROS1_PROJECT/vins-calib/devel/setup.bash;bash"

sleep 1s

# 第三布：启动键盘控制节点
gnome-terminal -t "start_BAG" -x bash -c " source /opt/ros/noetic/setup.bash;rosbag play /home/wyw/ROS1_PROJECT/BD/calib_data/wyw/2022-11-26-10-47-35.bag;bash"

sleep 30s
gnome-terminal -t "start_BAG" -x bash -c "gedit /home/wyw/ROS1_PROJECT/BD/lidar_imu_init/src/LiDAR_IMU_Init/result/lidar_imu_camera.txt"
