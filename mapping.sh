#! /bin/bash
# gnome-terminal -x bash -c "sleep 5"
# source ~/.bashrc
# cd /home/mechax/sentry_vision2
# colcon build --symlink-install
cmds=( 
	#"ros2 launch rm_bringup bringup.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 launch ig_lio_c map_mapping_launch.py"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch rm_navigation online_async_launch.py"
	"ros2 launch rm_navigation bringup_no_amcl_launch.py"
	"ros2 launch rm_decision my_launch.py"
	"ros2 launch rm_serial_driver serial_driver.launch.py"
	#"ros2 launch rm_vision_bringup vision_bringup.launch.py"
	)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
