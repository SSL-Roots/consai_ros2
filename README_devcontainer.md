# Devcontainer

consai_ros2の開発に必要なものが一発で起動する最高の開発環境

## Github Codespacesでの開発
Github Codespacesを使うと、Githubのクラウドマシン上でDevcontainerが立ち上がります。  

### ブラウザでの立ち上げ方
1. [consai_ros2のリポジトリ](https://github.com/SSL-Roots/consai_ros2/)を開きます。
1. 「<> Code」をクリックし、「Codespaces」をクリックします。
1. 「...」をクリックし「New with options...」をクリックします。
1. 一番下の「Machine type」を「4-core」に変更し「Create codespaces」をクリックします。
1. しばらく待ちます。
1. ブラウザ上でVSCodeのUIが出ればOK！

### VSCodeでの立ち上げ方
ローカルのVSCodeをGithubのクラウドにつなげて開発できます。

1. [ブラウザでの立ち上げ方](#ブラウザでの立ち上げ方)の手順に従って、一旦Codespacesを作成します。
1. [consai_ros2のリポジトリ](https://github.com/SSL-Roots/consai_ros2/)を開きます。
1. 「<> Code」をクリックし、「Codespaces」をクリックします。
1. 作成されたCodespace名の右にある「...」をクリックし、「Open in Visual Studio Code」をクリックします。
1. ローカルのVSCodeが起動し、クラウド上のCodespaceが開きます。

## ローカルでの開発
ローカルにcloneしたコードもDevcontainer上で開発できます。

1. VSCodeを起動します。
2. Ctrl + P →　「Dev Containers: Open Folder in Container...」をクリックします。
3. consai_ros2のフォルダを指定します。
4. しばらく待つとDevcontainerが起動します。


## つかいかた・注意点
### consai_ros2を起動する
`ros2 launch consai_examples start.launch.py game:=true yellow:=false invert:=false goalie:=0 vision_port:=10020 robot_control_port:=20011 robot_control_ip:='grsim' gui:=false`

`robot_control_ip` パラメータが `grsim` になっていることに注意してください。

### バックグラウンドで起動しているサービス
- grSim　(ポートフォワードなし)
- ssl-game-controller (8081番ポート)
- rosbridge-server (9090番ポート)
- consai_web-ui (3000番ポート)

#### consai_web_ui
Web版のCONSAI UIです。

VSCodeの「PORT」タブから3000番のポートに接続すると開きます。

Codespacesで動かしているときは、先にrosbridge-server (9090番ポート) を一度開かないとうまく動作しないので注意してください。


