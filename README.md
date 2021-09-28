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

２）センサの姿勢の変更  
```
rosrun lidar_with_mirror demo_change_angle.py
```

３）姿勢の推定（樋高君作成）  
```
rosrun lidar_with_mirror mirror_lidar.py
```
※左右のセンサの向きを90d度ずらしてから実行する  

### 実験

`experiment.py'を変更して，ロール方向もしくはピッチ方向にセンサを傾けるように変更

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
