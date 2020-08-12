# autoFlight

roslaunch vins_estimator realsense_fisheye.launch
roslaunch zed_wrapper zed_camera.launch
rosrun tf static_transform_publisher 0 0 3.15 0 0 base_link world 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 odom_vins world 100

