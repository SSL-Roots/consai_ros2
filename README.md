# CON-SAI 

CON-SAIはRoboCup SSLに 初めて参加する人でも開発できるサッカーAIです。
ROS 2で構成されています。

**CON-SAI** stands for **CON**tribution to *S*occer **AI**.

## Development

CON-SAIを開発する際にこの項目を読んでください。

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
