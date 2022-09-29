# lidar_with_mirror

以下のブランチが肥大化してきたのと，orne-boxに直接関わらない内容ですのででこちらで管理します．  
https://github.com/open-rdc/orne-box/tree/detect_low_obstacle

ビルド  
orne-boxの環境が構築されていれば，ビルドできると思います．

実行  

１）シミュレーション環境の実行  
```
roslaunch lidar_with_mirror_bringup lidar_with_mirror_sim.launch
```

２）センサの姿勢の変更（動作確認用）  
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
roslaunch lidar_with_mirror_sim.launch
rosrun lidar_with_mirror check_estimate_posture.py
rosrun lidar_with_mirror experiment.py
```

検出された地面の誤差の検証
```
roslaunch lidar_with_mirror_sim.launch
rosrun lidar_with_mirror check_ground_height.py
rosrun lidar_with_mirror experiment.py
```
