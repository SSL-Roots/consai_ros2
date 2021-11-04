# consai_examples

CON-SAIの使い方がわかるチュートリアルパッケージです。

## ロボットを動かす

次のコマンドを実行して、コントローラを起動します。

```sh
$ ros2 launch consai_robot_controller test.launch.py
```

別のターミナルで、pythonスクリプトを実行します。

```sh
$ ros2 run consai_examples control.py
```

### 解説

`control.py`は`consai_robot_controlelr`の使い方を知るためのスクリプトです。

`main()`内のコメントをON/OFFすることで、様々な動きを試せます。

```python
def main(target_is_yellow=False):
    # test_move_to()
    # test_move_to_normalized(3)
    # test_chase_ball()
    # test_chase_robot()
    # test_for_config_pid(test_theta=True)
    # test_shoot(1.0, 0.0)
    # test_pass_two_robots()
    test_pass_four_robots()
```

## ロボット、ボール情報を上下左右反転する

`consai_visition_tracker`の`invert`パラメータを`True`にすると、
ロボットとボールの情報を上下左右反転できます。

これは、試合プログラムを変更しなくてもコートチェンジできるため便利です。

`consai_vision_tracker`の`test.launch.py`を次のように書き換えます。

```diff
ComposableNode(
    package='consai_vision_tracker',
    plugin='consai_vision_tracker::Tracker',
    name='tracker',
-   parameters=[{'invert': False}],
+   parameters=[{'invert': True}],
    extra_arguments=[{'use_intra_process_comms': True}],
    ),
```
