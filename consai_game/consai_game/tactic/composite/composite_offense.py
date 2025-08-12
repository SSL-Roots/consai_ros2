# Copyright 2025 Roots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
条件に応じてキックやパスを切り替えるTactic
"""
import copy
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.core.tactic.composite_tactic_base import CompositeTacticBase
from consai_game.tactic.kick import Kick
from consai_game.tactic.receive import Receive
from consai_game.tactic.steal_ball import StealBall

from consai_game.world_model.field_model import Field

from consai_game.evaluation.evaluation import Evaluation
from consai_game.evaluation.evaluation_provider_node import EvaluationProviderNode
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tools
import math
from typing import Optional, Set


class SharedInfo:
    """CompositeOffenseの情報を共有するクラス"""

    def __init__(self):
        self.assigned_robot_ids: Set[int] = set()  # CompositeOffenseを担当しているロボットのID
        self.update_conter: int = 0  # 内部用更新カウンター
        self.can_control_ball_id: Optional[bool] = None  # ボールを操作できるロボットのID
        self.can_receive_ball_id: Optional[bool] = None  # ボールをレシーブできるロボットのID

    def register_robot(self, robot_id: int):
        """ロボットを登録する関数"""
        if robot_id not in self.assigned_robot_ids:
            self.assigned_robot_ids.add(robot_id)

    def unregister_robot(self, robot_id: int):
        """ロボットを登録解除する関数"""
        if robot_id in self.assigned_robot_ids:
            self.assigned_robot_ids.discard(robot_id)

    def update(self, world_model: WorldModel):
        """情報を更新する.

        外部でcounterを使い、多重更新を防ぐこと
        """
        # リセット
        self.can_control_ball_id = None
        self.can_receive_ball_id = None

        # ダブルタッチルールによるボールをけれないロボットのIDを取得
        prohibited_kick_id = world_model.robot_activity.our_prohibited_kick_robot_id

        # ボールを操作できるロボットを決める
        nearest_robot_id, next_nearest_robot_id = self.ball_nearest_robots(world_model)
        if nearest_robot_id == prohibited_kick_id:
            # ボール禁止ロボットがボールに一番近い場合
            # 2番目に近いロボットがボールを操作できる
            self.can_control_ball_id = next_nearest_robot_id
        else:
            self.can_control_ball_id = nearest_robot_id

        # ボールを受け取れるロボットを決める
        best_receiving_candidate, next_receiving_candidate = self.ball_receiving_candidates(world_model)
        if best_receiving_candidate == prohibited_kick_id:
            # ボール禁止ロボットがレシーブできる場合
            # 2番目にレシーブできるロボットがボールを扱う
            self.can_receive_ball_id = next_receiving_candidate
        else:
            self.can_receive_ball_id = best_receiving_candidate

    def ball_nearest_robots(self, world_model: WorldModel) -> tuple[int, Optional[int]]:
        """ボールに一番近いロボットと2番目に近いロボットを返す"""
        nearest = world_model.robot_activity.our_robots_by_ball_distance[0]
        next_nearest = None
        if len(world_model.robot_activity.our_robots_by_ball_distance) >= 2:
            next_nearest = world_model.robot_activity.our_robots_by_ball_distance[1]
        return nearest, next_nearest

    def ball_receiving_candidates(self, world_model: WorldModel) -> tuple[Optional[int], Optional[int]]:
        """ボールをレシーブできるロボットと2番目にレシーブできるロボットを返す"""
        nearest = None
        next_nearest = None
        if not world_model.ball_activity.ball_is_moving:
            # ボールが止まっている場合はレシーブできない
            return nearest, next_nearest

        best_receive_score = world_model.robot_activity.our_ball_receive_score[0]

        if best_receive_score.intercept_time == float("inf"):
            # ボールと交差しない場合はレシーブできない
            return nearest, next_nearest
        else:
            nearest = best_receive_score.robot_id

        if len(world_model.robot_activity.our_ball_receive_score) >= 2:
            next_best_receive_score = world_model.robot_activity.our_ball_receive_score[1]
            if next_best_receive_score.intercept_time != float("inf"):
                next_nearest = next_best_receive_score.robot_id

        return nearest, next_nearest


class CompositeOffense(CompositeTacticBase):
    shared_info = SharedInfo()

    def __init__(self, tactic_default: TacticBase, is_setplay=False, force_pass=False, kick_score_threshold=30):
        super().__init__(
            tactic_shoot=Kick(is_pass=False, is_setplay=is_setplay),
            tactic_pass=Kick(is_pass=True, is_setplay=is_setplay),
            tactic_tapping=Kick(is_tapping=True, is_setplay=is_setplay),
            tactic_receive=Receive(),
            tactic_steal=StealBall(),
            tactic_default=tactic_default,
        )

        self.kick_score_threshold = kick_score_threshold
        self.SHOOTING_MARGIN = 0

        self.evaluation: Evaluation = Evaluation()
        # self.field: Field = Field()
        # self.evaluation_provider_node: EvaluationProviderNode = EvaluationProviderNode()
        
        # 最初の動作を強制的にpassにするかの設定
        self.force_pass = force_pass

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)

        # ロボットのIDを登録する
        self.shared_info.register_robot(robot_id)

    def exit(self):
        super().exit()

        # ロボットのIDを登録解除する
        self.shared_info.unregister_robot(self.robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替えてrunする."""

        # 共有情報の更新
        if self.shared_info.update_conter != world_model.meta.update_counter:
            self.shared_info.update_conter = world_model.meta.update_counter
            self.shared_info.update(world_model)

        if self.robot_id == self.shared_info.can_receive_ball_id:
            return self.receive_the_ball(world_model)
        elif self.robot_id == self.shared_info.can_control_ball_id:
            # ボールに一番近い場合はボールを操作する
            return self.control_the_ball(world_model)
        # ボールを操作できない場合はデフォルトのtacticを実行する
        return self.run_sub_tactic(self.tactic_default, world_model)

    def control_the_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールを制御するためのTacticを実行する関数."""

        evaluation = self.evaluation

        # 相手がボールを持ってる場合は奪いに行く
        if world_model.ball_activity.is_their_team_ball_holder:
            # ボールを奪う
            return self.run_sub_tactic(self.tactic_steal, world_model)

        if (
            evaluation.kick_target.best_shoot_target.success_rate > self.kick_score_threshold - self.SHOOTING_MARGIN
            and self.force_pass is False
        ):
            # シュートできる場合かつforce_passがFalseの場合
            self.tactic_shoot.target_pos = evaluation.kick_target.best_shoot_target.pos
            # シュート相手がコロコロ切り替わらないようにマージンを設定
            self.SHOOTING_MARGIN = 20
            return self.run_sub_tactic(self.tactic_shoot, world_model)

        elif evaluation.kick_target.best_pass_target.success_rate > 30 or self.force_pass:
            # パスできる場合 か force_passがTrueの場合
            self.tactic_pass.target_pos = copy.deepcopy(evaluation.kick_target.best_pass_target.robot_pos)
            # パスターゲットの候補を探そうとしているのでシュートターゲットのマージンを0にする
            self.SHOOTING_MARGIN = 0

            return self.run_sub_tactic(self.tactic_pass, world_model)

        # TODO: 前進しつつ、敵がいない方向にドリブルしたい
        # シュート成功率が一番高いところに向かってドリブルする
        self.tactic_tapping.target_pos = evaluation.kick_target.best_shoot_target.pos
        return self.run_sub_tactic(self.tactic_tapping, world_model)

    def receive_the_ball(self, world_model: WorldModel) -> MotionCommand:
        """ボールをレシーブするためのTacticを実行する関数."""
        command = self.run_sub_tactic(self.tactic_receive, world_model)

        if world_model.ball_activity.ball_will_enter_their_goal:
            # ボールがゴールに入る場合は目標位置をシュートラインから外す
            command.desired_pose = self.modify_desired_pose_to_avoid_shoot(world_model, command.desired_pose)

        return command

    def modify_desired_pose_to_avoid_shoot(self, world_model: WorldModel, desired_pose: State2D) -> State2D:
        """シュートを避けるために目標位置を修正する関数."""
        DISTANCE_THRESHOLD = 0.3

        robot_pose = world_model.robots.our_visible_robots[self.robot_id].pos

        # ボールを中心に、ボール到着位置への座標系を作る
        trans = tools.Trans(
            world_model.ball.pos, tools.get_angle(world_model.ball.pos, world_model.ball_activity.ball_stop_position)
        )
        tr_stop_position = trans.transform(world_model.ball_activity.ball_stop_position)
        tr_desired_pose = trans.transform(desired_pose)
        tr_robot_pose = trans.transform(robot_pose)

        # desired_poseがボール軌道の範囲にない場合
        if tr_desired_pose.x < 0.0 or tr_desired_pose.x > tr_stop_position.x:
            return desired_pose

        # ボール軌道に接触していない場合
        if abs(tr_desired_pose.y) > DISTANCE_THRESHOLD:
            return desired_pose

        # ロボットが居る側に回避位置を生成する
        avoid_y = math.copysign(DISTANCE_THRESHOLD, tr_robot_pose.y)

        new_desired_pose = trans.inverted_transform(State2D(x=tr_desired_pose.x, y=avoid_y))
        new_desired_pose.theta = desired_pose.theta
        return new_desired_pose
