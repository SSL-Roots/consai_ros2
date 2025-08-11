from consai_game.world_model.world_model import WorldModel
from consai_game.core.play.play_condition import PlayCondition


class RobotConditions:
    """ロボットに関連する条件についてのクラス."""

    @staticmethod
    def their_robots_in_our_area_over_threshold() -> PlayCondition:
        """
        相手ロボットが指定台数以上自陣に侵入しているか判定する関数.
        """

        def condition(world_model: WorldModel) -> bool:
            """相手ロボットが指定台数以上自陣に侵入しているか判定する条件関数."""
            num = world_model.robot_activity.number_of_their_robots_in_our_area_over_threshold
            return 5 < num

        return PlayCondition(condition)
