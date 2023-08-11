# consai_robot_control_utils

ロボット制御の調整に使う各種ツールです。

## launchファイル

- `visualizer.launch.py` : 
  - ロボットの位置を可視化します。
  - GrSimのロボットを動かす際は必ず起動してください。

## speedcontrol_test

速度制御の確認に使います。  
CSVで定義した速度プロファイルに従い、ロボットに速度指令値を送ります。

### Usage

- `robot_id` : ロボットID
- `csv_path` : 速度プロファイルのCSVファイルのパス
- `loop` : trueで速度プロファイルを繰り返し実行する

```sh
# ビジュアライザを起動
$ ros2 launch consai_robot_control_utils visualizer.launch.py

# 速度プロファイルを再生する
$ cd consai_robot_control_utils/profile
$ ros2 run consai_robot_control_utils speedcontrol_test --ros-args \
-p robot_id:=1 \
-p csv_path:=$PWD/rotation_4sec.csv \
-p loop:=true
```
