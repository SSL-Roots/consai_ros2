# テストコードの実行方法

## パッケージをテストする

初回テスト時及び、パッケージごとに複数のテストを一括で実施したい場合に下記コマンドを実行します

```sh
$ cd ~/ros2_ws
# ワークスペースをビルド
$ colcon build --symlink-install

# パッケージをテスト
$ colcon test --packages-select consai_examples
# テスト結果を表示
$ colcon test-result --verbose
```

## テストファイル単体をテストする

ワークスペースのビルドが完了したら下記コマンドを実行します

```sh
# ワークスペースの読み込み
$ source ~/ros2_ws/install/setup.bash

# テストディレクトリへ移動
$ cd /path/to/tests

# テストを実行
$ python3 -m pytest test_role_assignment.py
```
