# Role Assignmentについて

RoleAssignmentノードの仕事は、ロボットの役割（ロール）を決めることです

## 仕様

### 設計

- RoleAssignmentノードはロール優先度をもとにロボットIDを管理します
  - `self._robot_id_of_role_priority`
  - インデックスが小さいほど、優先度が高いです
  - `self._robot_id_of_role_priority[0] == GOALIEのロボットID`と考えてOKです
- ノードはアクティブなロール一覧と、現時点でのロール一覧を管理します
  - `self._active_role_list`と`self._present_role_list`
  - 基本的にはそれぞれ同じ値を持ちますが、
ロボット交代時は`self._present_role_list`に`SUBSTITUTE`ロールが上書きされます

### 機能

- ノード初期化時の引数`goalie_id`よって`GOALIE`ロール担当者を固定します
- ノード初期化時の引数`our_team_is_yellow`よって自チームの色（blue or yellow）を設定します
- `TrackedFrame`型の`detection_tracked`トピックを受け取り、ロールを更新します
  - `TrackedFrame`にはフィールド上のロボット位置、ボール情報が含まれます
- `update_role()`関数でロール割り当てを更新します
- `update_role()`は担当するロールが変わったロボットIDリストを返します
- ロールの数よりフィールド上のロボットの数が多くてもエラーが発生しません
- フィールド上からロボットの台数が減っても、優先度の高いロールには常にだれかが割り当てられます
- ボールに一番近いロボットが`ATTACKER`ロールを担当します
- `GOALIE`がボールに一番近い場合は、二番目にボールに近いロボットが`ATTACKER`を担当します
- `update_role()`には、ボール位置ベースの`ATTACKER`選定を無効化できるフラグがあります
  - パスシュートでボールが移動している最中や、セットプレイなどで役立ちます
- `update_role()`には、ロボットの出場可能台数を設定するパラメータがあります
  - イエローカードやレッドカードでロボットを退避させる場合に有効です
- ロボットの出場可能台数が最大数より減った場合は、優先度の低いものから`SUBSTITUTE`ロールを割り当てます
- `SUBSTITUTE`が割り当てられた場合も、解除された場合も、担当に変化があったとみなして`update_role()`はそのロボットIDを返します
