from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.world_model.world_model import WorldModel
from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tools


class CenterBack(TacticBase):
    """
    ディフェンスエリアに沿って守るセンターバック用タクティック
    ボールとゴール中央を結んだ直線と、ディフェンスエリアの水平線との交点を基準に動作
    """

    def __init__(self, area_margin: float = 0.2, index: int = 0, total: int = 1):
        """
        Args:
            area_margin: ディフェンスエリアから離れる距離[m]
            index: 担当ロボットの並び順（0, 1, 2, ...）
            total: 担当ロボットの総数
        """
        super().__init__()
        self.area_margin = area_margin
        self.index = index
        self.total = total

    def _calc_alignment_positions(self, base, robot_size, total):
        """
        baseを中心に、robot_sizeの幅で均等に分散した位置の配列を返す

        Args:
            base: 中心位置
            robot_size: ロボット間の間隔
            total: ロボットの総数

        Returns:
            List[float]: 分散した位置の配列
        """
        if total == 1:
            return [base]

        # 中心からのオフセットを計算
        # 例: total=3の場合、[-1, 0, 1] * (robot_size/2)
        offsets = [(i - (total - 1) / 2) * robot_size for i in range(total)]
        return [base + offset for offset in offsets]

    def run(self, world_model: WorldModel) -> MotionCommand:
        command = MotionCommand()
        command.robot_id = self.robot_id
        robot_size = 0.2

        # ディフェンスエリアの外側ラインのx座標
        field = world_model.field
        field_points = world_model.field_points
        # ディフェンスエリアの外側ライン
        defense_x_with_margin = field_points.our_defense_area.top_right.x + self.area_margin
        defense_y_with_margin_top = field_points.our_defense_area.top_left.y + self.area_margin
        defense_y_with_margin_bottom = field_points.our_defense_area.bottom_left.y - self.area_margin

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        # ゴール中央の位置
        goal_center = State2D(x=-field.half_length, y=0.0)

        # ディフェンスエリアの水平線の両端
        defense_line_top_right = State2D(x=defense_x_with_margin, y=defense_y_with_margin_top)
        defense_line_bottom_right = State2D(x=defense_x_with_margin, y=defense_y_with_margin_bottom)
        defense_line_top_left = State2D(x=-field.half_length, y=defense_y_with_margin_top)
        defense_line_bottom_left = State2D(x=-field.half_length, y=defense_y_with_margin_bottom)

        # ボールとゴール中央を結んだ線と、ディフェンスエリアの水平線の交点を計算
        intersection_right = tools.get_line_intersection(
            ball_pos, goal_center, defense_line_top_right, defense_line_bottom_right
        )
        intersection_top = tools.get_line_intersection(
            ball_pos, goal_center, defense_line_top_right, defense_line_top_left
        )
        intersection_bottom = tools.get_line_intersection(
            ball_pos, goal_center, defense_line_bottom_right, defense_line_bottom_left
        )

        base_x = None
        base_y = 0.0
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
            if self.total > 1:
                x_positions = self._calc_alignment_positions(base_x, robot_size, self.total)
                if y < 0:
                    x = x_positions[self.index]
                else:
                    x = x_positions[self.total - self.index - 1]
                if x > defense_x_with_margin:
                    diff = x - defense_x_with_margin
                    x = defense_x_with_margin
                    y = y - diff if y > 0 else y + diff
            else:
                x = base_x
        else:
            x = defense_x_with_margin
            # 複数台で均等にy方向に分散
            if self.total > 1:
                y_positions = self._calc_alignment_positions(base_y, robot_size, self.total)
                y = y_positions[self.index]
                if y > defense_y_with_margin_top:
                    diff = y - defense_y_with_margin_top
                    y = defense_y_with_margin_top
                    x = x - diff
                elif y < defense_y_with_margin_bottom:
                    diff = defense_y_with_margin_bottom - y
                    y = defense_y_with_margin_bottom
                    x = x - diff
            else:
                y = base_y

        command.mode = MotionCommand.MODE_NAVI
        command.desired_pose.x = x
        command.desired_pose.y = y
        command.desired_pose.theta = 0.0

        # TODO: このあたりは試合でイエローカードもらってたりしたら調整
        command.navi_options.avoid_our_robots = False
        command.navi_options.avoid_pushing = False

        return command
