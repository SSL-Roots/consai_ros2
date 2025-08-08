from consai_game.world_model.world_model import WorldModel
from consai_game.core.play.play_condition import PlayCondition


class RobotConditions:
    """ロボットに関連する条件をまとめたクラス."""

    @staticmethod
    def their_robots_in_our_area() -> PlayCondition:
        """
        相手ロボットの台数の半分以上が自エリアに侵入しているか判定する関数.
        """

        def condition(world_model: WorldModel) -> bool:
            """ボールの速度が閾値より小さいかを判定する条件関数."""
            num = world_model.robot_activity.number_of_their_robots_in_our_area
            return 5 < num

        return PlayCondition(condition)
