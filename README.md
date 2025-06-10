#2025/6/10 起動初期化位置変更gui、lidarとgnss切り替え、zed-f9pパラメータ更新、駆動ソフトでZ時刻データ処理取り消す
--------------------------------------------------------------------------------------------------------

1.初期位置処理gui追加、ルート生成、自律走行に初期位置選択機能追加

2.gnss制御で出発時、到着時にlidarの向きを借用、lidar->gnssに切替時にlidarの向きを借用、gnss->lidarに切替時にgnssデータでlidar位置更新

3.zed-f9pパラメータでbaudrateと出力内容を減少に変更

4.駆動ソフトでz軸のデータを無視される

#2025/6/3 cuda(GPU)で自己位置推定に変更、GNSS/RTK基準局情報追加、顔認識をCPUからCUDA(GPU）に変更、音声処理をCPUに変更
----------------------------------------------------------------------------------------------------------
1.CUDA(GPU)で自己位置推定の環境：ubuntu24.04LTS + ROS2 jazzy + nvidia driver 550 + cuda11.8 + gcc(g++)V11

2.cpuで自己位置推定最大速度：0.4m/s => GPUで自己位置推定最大速度：0.8m/s,adaFace(gpu)付け

3.音声処理cpuに変更、whisperでcuda12.x以上必要のため、cuda11.8と兼用できない

4.hdl_localizationのパラメータ変更、安定に位置推定のため

#2025/5/23 gnss内容表示更新、zed-f9pコンフィグ更新、座標系原点一致しない不具合修正、音声wakeupできない不具合修正
-----------------------------------------------------------------------------------------------------
１．zed-f9p出力1hz=>10hzに変更、ntrip baudrate 38400=>115200に変更

2. gpsからbase_linkへの座標変換の不具合修正
   
３．音声環境のノイズのレベルを変な値を取る不具合を修正

#2025/5/19 GNSSで自律走行、顔認識、音声認識と処理
--------------------------------------------------------------------------
１．gnssで地図生成時に座標一致できた

２．音声制御、人間認識と制御、顔認識と制御機能追加

３．走行ルールでの制御機能追加

＃2025/4/23 gnssで屋内、屋外から地図生成機能追加
--------------------------------------------------------------------------
1.guiにgnss検証用の画面追加

2.屋内でgnssの座標原点とロボット向きの確定方法追加

#2025/4/18 gnssで地図生成機能追加
--------------------------------------------------------------------------
1.gnssのNtripにつながる処理追加

2.ntripのコンフィグgui画面追加

3.Lidar内蔵6軸imuのyaw静止誤差18degを校正値設定：mapping_navsat.yaml yaw_offset: 0.314159

#2025/4/15 音声認識会話(RAG）機能追加、画像認識先導機能追加
---------------------------------------------------------------------------
１．chatbotフォルダ追加、音声処理会話、英語と中国語を追加

２．画像認識、先導機能追加

#2025/4/9 音声で提示処理追加、回転後と出発時の間隔時間延長、先導制御できない不具合修正
----------------------------------------------------------------------------

#2025/4/7 自律走行GUI、障害物検知、走行ルール設置、画像認識と先導機能
------------------------------------------------------------------------
1.自律走行gui，ルート生成gui追加

2.走行ルール設定追加

3.画像認識制御追加

# 2025/3/24 hdl_localization 自己位置推定移植
------------------------------------------------------------------------
1.自己位置推定動作確認した


#2025/3/18 robot_gui画面操作追加、Liosam地図生成機能更新
--------------------------------------------------------------------------
1.MainWindow、MakeMap、MakeRoute GUI追加

2.liosamのframe＿id更新、パラメータ更新、地図不安定不具合修正

# amr-3d-lidar-ros2
mikuni amr for ubuntu 24.04 lts, ros2 jazzy by 3d lidar sensor

#2024/11/5

-livox mid-360の処理をlio-samに入れた

-3d地図生成：lio-samパッケージの３d点群地図不具合修正

-2d地図：slam_toolboxパッケージでlaunchファイルを追加

-地図生成コマンド流れ(ターミナル)：

  １) $ ros2 launch amr_ros om_map_liosam_3D.launch.py
  
  2) $ ros2 launch amr_ros om_map_liosam_2D.launch.py
     
  4) $ ros2 launch amr_ros om_manual.launch.py
     
　注意：Livox_SDK2のソースコードはビルとする際に下記の修正必要：
 
 　・sdk_core/comm/define.h >>追加：#include <cstdint>
  
   ・sdk_core/logger_handler/file_manager.h >>追加：#include <cstdint>　

#2024/10/29
- slam_toolboxパッケージインストール：$ sudo apt install ros-jazzy-slam-toolbox
- amr_rosパッケージ作成、手動制御と地図生成launchファイル増加
  
  手動制御（※検証済み)：$ros2 lanuch amr_ros om_manual.launch.py
  
  地図生成(※検証中)：$ros2 launch amr_ros om_map_liosam.launch.py
  
- 新規om_cartパッケージ
- 新規pointcloud_to_laserscanパッケージ：

#2024/10/22 1st init
- add livox driver2
- add oriental motor driver
- add lio-sam package without mid-360/GPS function: eigen v3.3.7, gtsam v4.2 shoud be setup first
           
