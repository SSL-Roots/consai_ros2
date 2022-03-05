# テスト

## テストの実行方法

```sh
$ cd ros2_ws
# パッケージを選択してビルドを実行
$ colcon build --symlink-install --packages-select consai_robot_controller

# パッケージを選択してテストを実行
$ colcon test --packages-select consai_robot_controller

# テスト結果の表示
$ colcon test-result --verbose
```
