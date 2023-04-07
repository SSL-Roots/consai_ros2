# DockerでCON-SAIを動かす

CON-SAIのDockerイメージは
[SSL-Roots/consai_ros2/pkgs/container/consai_ros2](https://github.com/SSL-Roots/consai_ros2/pkgs/container/consai_ros2)
に公開されています。

イメージ名は`ghcr.io/ssl-roots/consai_ros2:main`です。

## CON-SAIを動かす

GUIアプリを起動するため、
[osrf/rocker](https://github.com/osrf/rocker)
を使用することを推奨します

```sh
# rockerのインストール
$ sudo apt install python3-rocker
```

rockerのオプションには、
ホストのネットワーク環境を使用するための`--net=host`と
[ネットワーク使用時のエラー](https://github.com/osrf/rocker/issues/13)
を回避するための`--privileged`を与えます。

またROSは通信に共有メモリを使用するため、`/dev/shm`をコンテナにマウントします。

次のコマンドは`start.launch.py`を起動する例です

```sh
$ rocker --x11 --net=host --privileged --volume /dev/shm:/dev/shm -- ghcr.io/ssl-roots/consai_ros2:main ros2 launch consai_examples start.launch.py
```

## DockerでCON-SAIを開発する

ローカルにCON-SAIパッケージをクローンし、それをDockerコンテナにマウントして作業することを推奨します。
ローカルでのファイル変更がコンテナにも反映され、
コンテナ内での作業（例：`colcon build`によるワークスペースのビルド）もローカルに反映されます。

```sh
# ワークスペースの作成
$ mkdir -p ~/consai_ws/src
$ cd ~/consai_ws/src

# CON-SAIと依存パッケージをクローン
$ git clone https://github.com/SSL-Roots/consai_ros2.git
$ git clone https://github.com/SSL-Roots/consai_frootspi_msgs.git

# 以下、どのディレクトリでコマンドを実行してもOKです
# --volumeオプションでワークスペースをマウントしてコンテナを起動
$ rocker --x11 --net=host --privileged --volume /dev/shm:/dev/shm ~/consai_ws:/root/ros2_ws -- ghcr.io/ssl-roots/consai_ros2:main
# 例：パッケージをビルドする
$ rocker --x11 --net=host --privileged --volume /dev/shm:/dev/shm ~/consai_ws:/root/ros2_ws -- ghcr.io/ssl-roots/consai_ros2:main colcon build --symlink-install
# 例：CON-SAIを起動する
$ rocker --x11 --net=host --privileged --volume /dev/shm:/dev/shm ~/consai_ws:/root/ros2_ws -- ghcr.io/ssl-roots/consai_ros2:main ros2 launch consai_examples start.launch.py
```

## Dockerイメージをビルドする

ローカルでもDockerイメージをビルドできます。
Dockerfileを編集したい場合におすすめです。

`build.sh`の引数には`$ROS_DISTRO（例：humble）`を与えます。
成功すると`consai:humble`という名前のイメージが生成されます。

```sh
$ cd consai_ros2/.docker
$ ./build.sh 
...
Summary: 10 packages finished [1min 58s]
Removing intermediate container bd43dc976256
 ---> b5a4f3477119
Step 9/9 : COPY ./ros_entrypoint.sh /ros_entrypoint.sh
 ---> 2707a311afa4
Successfully built 2707a311afa4
Successfully tagged consai:humble
```

## その他

### 環境変数の渡し方

`--env`オプションを使用します

```sh
$ rocker --env ROS_DOMAIN_ID=22 --x11 --net=host --privileged --volume /dev/shm:/dev/shm ~/consai_ws:/root/ros2_ws -- ghcr.io/ssl-roots/consai_ros2:main
```
