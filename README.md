# amr-3d-lidar-ros2
mikuni amr for ubuntu 24.04 lts, ros2 jazzy by 3d lidar sensor

#2024/10/22 1st init
- add livox driver2
- add oriental motor driver
- add lio-sam package without mid-360/GPS function: eigen v3.3.7, gtsam v4.2 shoud be setup first
                                 
#2024/10/29
- slam_toolboxパッケージインストール：$ sudo apt install ros-jazzy-slam-toolbox
- amr_rosパッケージ作成、手動制御と地図生成launchファイル増加
  手動制御（※検証済み)：$ros2 lanuch amr_ros om_manual.launch.py
  地図生成(※検証中)：$ros2 launch amr_ros om_map_liosam.launch.py
- 新規om_cartパッケージ
- 新規pointcloud_to_laserscanパッケージ：
