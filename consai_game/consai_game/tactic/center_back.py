from typing import Set
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tools


class CenterBackAssignment:
    def __init__(self):
        self.assigned_robots: Set[int] = set()

    @property
    def total(self) -> int:
        """登録されているロボットの総数"""
        return len(self.assigned_robots)

    def get_index(self, robot_id: int) -> int:
        """
        指定されたロボットIDのインデックスを取得

        Args:
            robot_id: ロボットID

        Returns:
            int: ロボットIDのインデックス（0から始まる）
        """
        if robot_id not in self.assigned_robots:
            return 0
        return sorted(list(self.assigned_robots)).index(robot_id)

    def register_robot(self, robot_id: int):
        """
        ロボットを登録する関数

        Args:
            robot_id: 登録するロボットID
        """
        self.assigned_robots.add(robot_id)

    def unregister_robot(self, robot_id: int):
        """
        ロボットを登録解除する関数

        Args:
            robot_id: 登録解除するロボットID
        """
        self.assigned_robots.discard(robot_id)


class CenterBack(TacticBase):
    """
    ディフェンスエリアに沿って守るセンターバック用タクティック
    ボールとゴール中央を結んだ直線と、ディフェンスエリアの水平線との交点を基準に動作
    """

    assignment_module = CenterBackAssignment()

    def __init__(self):
        """
        Args:
            area_margin: ディフェンスエリアから離れる距離[m]
            index: 担当ロボットの並び順（0, 1, 2, ...）
            total: 担当ロボットの総数
        """
        super().__init__()
        self.area_margin = 0.2  # ディフェンスエリアから離れる距離[m]
        self.robot_size = 0.2  # ロボット同士が離れる距離[m]
        self.ball_move_threshold = 0.5  # ボールが動いているか判定する閾値
        # ボールが速度を持っているときのディフェンスエリア拡張用
        self.receive_defense_area_margin = 1.0

    def exit(self):
        super().exit()
        self.assignment_module.unregister_robot(self.robot_id)

    def _calc_alignment_positions(self, base, robot_size, total, limit=None):
        """
        baseを中心に、robot_sizeの幅で均等に分散した位置の配列を返す
        limitを超える場合は、limitを限界として配置を調整する

        Args:
            base: 中心位置
            robot_size: ロボット間の間隔
            total: ロボットの総数
            limit: 下限値（マイナス値 or None）

        Returns:
            List[float]: 分散した位置の配列
        """
        if total == 1:
            return [base]

        # 中心からのオフセットを計算
        # 例: total=3の場合、[-1, 0, 1] * (robot_size/2)
        offsets = [(i - (total - 1) / 2) * robot_size for i in range(total)]
        positions = [base + offset for offset in offsets]

        # limitを超える位置がある場合、配置を調整
        if limit is not None and min(positions) < limit:
            # 最小値がlimitになるように全体をシフト
            shift = limit - min(positions)
            positions = [pos + shift for pos in positions]

        return positions

    def run(self, world_model: WorldModel) -> MotionCommand:
        # ロボットを登録
        self.assignment_module.register_robot(self.robot_id)
        command = MotionCommand()
        command.robot_id = self.robot_id

        index = self.assignment_module.get_index(self.robot_id)
        total = self.assignment_module.total

        # ディフェンスエリアの外側ラインのx座標
        field = world_model.field
        field_points = world_model.field_points
        # ディフェンスエリアの外側ライン
        defense_x_with_margin = field_points.our_defense_area.top_right.x + self.area_margin
        defense_y_with_margin_top = field_points.our_defense_area.top_left.y + self.area_margin
        defense_y_with_margin_bottom = field_points.our_defense_area.bottom_left.y - self.area_margin

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        next_ball_pos = world_model.ball_activity.next_ball_pos
        # ボールの速度を取得
        ball_vel = world_model.ball.vel
        # ゴール中央の位置
        goal_center = State2D(x=-field.half_length, y=0.0)
        target_pos = goal_center

        # ディフェンスエリアの線分たち
        defense_line_top_right = State2D(x=defense_x_with_margin, y=defense_y_with_margin_top)
        defense_line_bottom_right = State2D(x=defense_x_with_margin, y=defense_y_with_margin_bottom)
        defense_line_top_left = State2D(x=-field.half_length, y=defense_y_with_margin_top)
        defense_line_bottom_left = State2D(x=-field.half_length, y=defense_y_with_margin_bottom)

        # ボールが速度を持っているときのディフェンスエリア拡張用
        defense_line_top_left_with_margin = State2D(
            x=-field.half_length, y=defense_y_with_margin_top + self.receive_defense_area_margin
        )
        defense_line_bottom_left_with_margin = State2D(
            x=-field.half_length, y=defense_y_with_margin_bottom - self.receive_defense_area_margin
        )

        # ボールが速度を持っている＆ゴールへ向かっている
        if ball_vel.x < -self.ball_move_threshold:
            # ボールの行く先がディフェンスエリア + マージンにあるかどうかを判定
            intersection_left = tools.get_line_intersection(
                ball_pos,
                next_ball_pos,
                defense_line_top_left_with_margin,
                defense_line_bottom_left_with_margin,
                is_on_line1_check=False,
            )
            if intersection_left is not None:
                target_pos = intersection_left

        # ボールとターゲット（ゴール中央 or ボールの行く先）を結んだ線と、ディフェンスエリアの水平線の交点を計算
        # ボールとターゲットを結んだ線は線分ではなく直線で判定する
        intersection_right = tools.get_line_intersection(
            ball_pos, target_pos, defense_line_top_right, defense_line_bottom_right, is_on_line1_check=False
        )
        intersection_top = tools.get_line_intersection(
            ball_pos, target_pos, defense_line_top_right, defense_line_top_left, is_on_line1_check=False
        )
        intersection_bottom = tools.get_line_intersection(
            ball_pos, target_pos, defense_line_bottom_right, defense_line_bottom_left, is_on_line1_check=False
        )

        base_x = None
        base_y = 0.0

        # ボールがフィールド内にあることを確認
        if not world_model.ball_position.is_outside():
            if intersection_right is not None:
                base_y = intersection_right.y
            else:
                # ボールが上半分にある場合
                if intersection_top is not None:
                    base_x = intersection_top.x
                # ボールが下半分にある場合
                if intersection_bottom is not None:
                    base_x = intersection_bottom.x

        if base_x is not None:
            y = defense_y_with_margin_top if intersection_top is not None else defense_y_with_margin_bottom
            # 複数台で均等にx方向に分散
            x_positions = self._calc_alignment_positions(base_x, self.robot_size, total, -field.half_length + 0.1)
            if y < 0:
                x = x_positions[index]
            else:
                # 並び順が逆になるため、indexを反転
                x = x_positions[total - index - 1]
            # はみ出たとき
            if x > defense_x_with_margin:
                diff = x - defense_x_with_margin
                x = defense_x_with_margin
                y = y - diff if y > 0 else y + diff

        else:
            x = defense_x_with_margin
            # 複数台で均等にy方向に分散
            y_positions = self._calc_alignment_positions(base_y, self.robot_size, total)
            y = y_positions[index]
            # はみ出たとき
            if y > defense_y_with_margin_top:
                diff = y - defense_y_with_margin_top
                y = defense_y_with_margin_top
                x = x - diff
            elif y < defense_y_with_margin_bottom:
                diff = defense_y_with_margin_bottom - y
                y = defense_y_with_margin_bottom
                x = x - diff

        command.mode = MotionCommand.MODE_NAVI
        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        # TODO: このあたりは試合でイエローカードもらってたりしたら調整
        command.navi_options.avoid_our_robots = False
        command.navi_options.avoid_pushing = False

        return command
