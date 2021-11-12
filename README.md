# CON-SAI 

CON-SAIはRoboCup SSLに 初めて参加する人でも開発できるサッカーAIです。
ROS 2で構成されています。

**CON-SAI** stands for **CON**tribution to *S*occer **AI**.

## Requirements

- Linux OS
    - Ubuntu 20.04 tested and is recommended
- ROS 2
    - [Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
    - [colcon build tool](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
- RoboCup SSL Official Softwares
    - [grSim](https://github.com/RoboCup-SSL/grSim)
    - [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)

## Installation

### Binary installation

TBD

### Source build

```sh
# Setup ROS environment
$ source /opt/ros/foxy/setup.bash

# Create working directory
$ mkdir -p ~/ros2_ws/src
# Download consai_ros2
$ cd ~/ros2_ws/src
$ git clone https://github.com/SSL-Roots/consai_ros2.git

# Install dependencies
$ rosdep install -r -y -i --from-paths .

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
# Setup working directory's environment
$ source ~/ros2_ws/install/setup.bash
```

## Quick start

```sh
# Start grSim and ssl-game-controller, then
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch consai_examples start.launch.py

# anothor terminal
$ source ~/ros2_ws/install/setup.bash
$ ros2 run consai_examples control_by_referee.py
```

CON-SAIの使い方は[consai_examplesのREADME](./consai_examples/README.md)を参照してください。

## Packages

- consai
    - メタパッケージ
- consai_examples
    - CON-SAIの各種パッケージを使ったサンプル集です
- consai_msgs
    - CON-SAIで使用するデータ型を定義するパッケージです
- consai_observer
    - フィールド情報を解析するパッケージです
- consai_robot_controller
    - ロボットの走行、キック、ドリブル制御を担うパッケージです
- consai_vision_tracker
    - ビジョン情報をフィルタリングするパッケージです
- consai_visualizer
    - ビジョン情報やロボットの走行情報を描画するパッケージです
- robocup_ssl_comm
    - SSL-Vision、SSL-Game-Controller、grSimのデータパケットをROS 2のトピックに変換するパッケージです
- robocup_ssl_msgs
    - SSL-Vision、SSL-Game-Controller、grSimのデータプロトコルをROS 2のデータ型に再定義するパッケージです

## License

(C) 2021 Roots

各ファイルはファイル内に明記されているライセンスに従います。
ライセンスが明記されていない場合は、Apache License, Version 2.0に従います。
ライセンスの全文は[LICENSE](./LICENSE)から確認できます。

## Development

CON-SAIを開発する際にこの項目を読んでください。

### 開発方針について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません
- Issue、Pull Requestは常に受け付けていますが、チームの開発方針を優先するため、提案が受け入れられない場合があります

### Lint

コードの見た目を整えるためにlintでチェックしています。

下記コマンドを実行して、チェックを実行してください。

```sh
$ cd ~/ros2_ws
# 全てのパッケージのテストを実行
$ colcon test
# あるいは、パッケージ名指定でテストを実行
$ colcon test --packages-select robocup_ssl_comm 
# テスト結果を表示
$ colcon test-result --verbose
```

C++のコードは`ament_uncrustify`を使って、自動で整形できます。

```sh
# フォーマットのチェック
$ ament_uncrustify robocup_ssl_comm/src/vision_component.cpp
# --reformat オプションで自動整形
$ ament_uncrustify --reformat robocup_ssl_comm/src/vision_component.cpp
```

Pythonのコードは`ament_flake8`を使って、フォーマットとチェックできます。
自動整形はできません。

```sh
$ ament_flake8 consai_examples/consai_examples/control.py
```

外部ツール（例：autopep8）を使えば自動整形できます。
[ROS 2のPythonコードスタイル](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html#python)
に沿うようにパラメータを設定してください。

```sh
$ sudo apt install python3-autopep8

# 1行の100文字に制限
# -i オプションで自動整形
$ autopep8 --max-line-length 99 -i consai_examples/consai_examples/control.py
```
