
# consai_description

consaiで使用するパラメータを定義したパッケージです。

パッケージ間で共通のパラメータを扱うために、`parameter_publisher`ノードを提供します。

## Usage

`parameter_publisher.launch.py`を起動します。

`ros2 launch consai_description parameter_publisher.launch.py`

起動すると、`config`ディレクトリのパラメータが`parameter_publisher`ノードにセットされます。

```bash
$ ros2 param list
/parameter_publisher:
  rule.ball.diameter
  rule.field.goal_width
  rule.field.length
  rule.field.penalty_depth
  rule.field.penalty_width
  rule.field.width
  rule.keep_outs.distance_to_ball_at_stop
  rule.robots.max_diameter
  rule.robots.max_height
  rule.robots.max_num_in_field
  rule.robots.num_of_ids
  strategy.div_a_field.ball_diameter
  strategy.div_a_field.length
  strategy.div_a_field.robot_diameter
  strategy.div_a_field.width
  use_sim_time
```

また、パラメータはJSON形式で`/consai_param/**`としてパブリッシュされます。

```bash
$ ros2 topic list
/consai_param/rule
/consai_param/strategy

$ ros2 topic echo --once /consai_param/rule
data: '{"field": {"length": 12.0, "width": 9.0, "goal_width": 1.8, "penalty_depth": 1.8, "penalty_width": 3.6}, "ball": {"diameter": 0....'
---
```

このトピックをデコードすることで、パラメータを取得できます。

`parameter_publisher`ノードのパラメータを変更すると、トピックの内容も書き換わります。

## Config files

- rule/division_a.yaml: Division Aのルールを定義します
- strategy/normal.yaml: consaiの通常戦略パラメータを定義します
