# lidar_with_mirror  
ミラー搭載LiDARの検証用プログラム  

以下のブランチの引継ぎ  
https://github.com/open-rdc/orne-box/tree/detect_low_obstacle  

## Install  
Precondition:  
Install ROS noetic (Ubuntu20.04/docker)  
Install python3-catkin-tools  

```
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/open-rdc/lidar_with_mirror
wstool init
wstool merge lidar_with_mirror/lidar_with_mirror_pkgs.install
wstool up
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

### Execution 

1) launch simulator  
```
roslaunch lidar_with_mirror_bringup lidar_with_mirror_sim.launch
```

2) センサの姿勢の変更（動作確認用）  
```
rosrun lidar_with_mirror demo_change_angle.py
```

関節配置  
<img src="https://user-images.githubusercontent.com/5755200/191669025-2a382114-529b-44cf-bc44-abda95df3f5a.png" width="300">

実行の様子  
[![IMAGE](http://img.youtube.com/vi/xApM7J0YAwk/0.jpg)](https://youtu.be/xApM7J0YAwk)

### 実験

`check_estimate_posture.py`　姿勢の推定
`check_ground_height.py` 

`experiment.py`　を変更して，ロール方向もしくはピッチ方向にセンサを傾けるように変更

推定した姿勢の誤算の検証
```
roslaunch lidar_with_mirror_bringup lidar_with_mirror_sim.launch
rosrun lidar_with_mirror check_estimate_posture.py
rosrun lidar_with_mirror experiment.py
```

検出された地面の誤差の検証
```
roslaunch lidar_with_mirror_bringup lidar_with_mirror_sim.launch
rosrun lidar_with_mirror check_ground_height.py
rosrun lidar_with_mirror experiment.py
```

### Install python3-catkin-tools  

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install python3-catkin-tools
```
