# consai_robot_control_utils

ロボット制御の調整に使う各種ツールです。

## speedcontrol_test
速度制御の確認に使います。  
CSVで定義した速度プロファイルに従い、ロボットに速度指令値を送ります。

### Usage
`ros2 run consai_robot_control_utils speedcontrol_test --ros-args -p robot_id:=<ロボットID> -p csv_path:=<プロファイルCSVのパス> -p loop:=<プロファイルの実行を繰り返すか>`

ex:  
`ros2 run consai_robot_control_utils speedcontrol_test --ros-args -p robot_id:=11 -p csv_path:=/home/roots/ros2_ws/src/consai_ros2/consai_robot_control_utils/profile/rotation_4sec.csv -p loop:=true`