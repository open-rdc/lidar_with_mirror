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
