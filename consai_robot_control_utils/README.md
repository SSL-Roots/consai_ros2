# consai_robot_control_utils

ロボット制御の調整に使う各種ツールです。

## launchファイル

- `visualizer.launch.py` : 
  - ロボットの位置を可視化します。
  - GrSimのロボットを動かす際は必ず起動してください。
- `rqt_plot.launch.py`:
  - ロボットの速度指令値`/robot*/command`とローカル速度`/robot_local_velocities`をrqtp_plotで表示します。

## speedcontrol_test

速度制御の確認に使います。  
CSVで定義した速度プロファイルに従い、ロボットに速度指令値を送ります。

### Usage

- `robot_id` : ロボットID
  - type : int
  - default : 0
- `yellow` : trueで黄色チーム、falseで青色チーム
  - type : bool
  - default : false
- `csv_path` : 速度プロファイルのCSVファイルのパス
  - type : string
  - default : ""
- `loop` : trueで速度プロファイルを繰り返し実行する
  - type : bool
  - default : false

```sh
# ビジュアライザを起動
$ ros2 launch consai_robot_control_utils visualizer.launch.py

# 速度プロファイルを再生する
$ cd consai_robot_control_utils/profile
$ ros2 run consai_robot_control_utils speedcontrol_test --ros-args \
-p robot_id:=0 \
-p yellow:=false \
-p csv_path:=$PWD/rotation_4sec.csv \
-p loop:=true
```
