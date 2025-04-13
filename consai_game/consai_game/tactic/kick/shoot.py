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

from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase, TacticState
from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool
import numpy as np

from transitions import Machine

BALL_NEAR_THRESHOLD = 0.5  # ボールが近いとみなす距離の閾値[m]
SHOOT_ANGLE_THRESHOLD = 10  # シュート角度の閾値[degree]

class ShootStateMachine(Machine):
    """シュート状態遷移マシン."""

    def __init__(self, name):
        self.name = name
        self.move_pos = State2D()
        
        # 状態定義
        states = ['chasing', 'aiming', 'shooting']

        # 遷移定義
        transitions = [
            { 'trigger': 'ball_near',  'source': 'chasing',  'dest': 'aiming' },
            { 'trigger': 'ball_far',   'source': 'aiming',   'dest': 'chasing' },
            { 'trigger': 'shoot',      'source': 'aiming',   'dest': 'shooting' },
            { 'trigger': 'done_shooting', 'source': 'shooting', 'dest': 'chasing' },
            { 'trigger': 'reset',      'source': '*',        'dest': 'chasing' }
        ]

        # ステートマシン構築
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='chasing')


    def on_enter_chasing(self):
        print(f"{self.name} is now chasing the ball.")


    def on_enter_aiming(self):
        print(f"{self.name} is now aiming at the ball.")


    def on_enter_shooting(self):
        print(f"{self.name} is shooting!")


class Shoot(TacticBase):
    """指定した位置にシュートするTactic."""

    def __init__(self, target_x=6.0, target_y=0.0):
        super().__init__()
        self.target_pos = State2D()
        self.target_pos.x = target_x
        self.target_pos.y = target_y
        self.move_pos = State2D()

        self.robot = ShootStateMachine("robot")


    def reset(self, robot_id: int) -> None:
        self.robot_id = robot_id
        self.state = TacticState.RUNNING
        self.robot.reset()


    def run(self, world_model: WorldModel) -> MotionCommand:
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        # ボールの位置を取得
        ball_pos = world_model.ball.pos
        robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

        # ロボットとボールの距離を計算
        dist_to_ball = tool.get_distance(ball_pos, robot_pos)

        # シュートの角度を計算
        shoot_angle = tool.get_angle(ball_pos, self.target_pos)

        if self.robot.state == 'chasing' and dist_to_ball <= BALL_NEAR_THRESHOLD:
            # ボールが近く、状態がchasingの場合、シュートの準備をする
            self.robot.ball_near()
            self.move_pos.x = ball_pos.x - 0.1 * np.cos(shoot_angle)
            self.move_pos.y = ball_pos.y - 0.1 * np.sin(shoot_angle)
            self.move_pos.theta = shoot_angle
        
        elif self.robot.state == 'chasing' and dist_to_ball > BALL_NEAR_THRESHOLD:
            # ボールが遠く、状態がchasingの場合、ボールを追いかける
            self.move_pos.x = ball_pos.x - 0.5
            self.move_pos.y = ball_pos.y
            self.move_pos.theta = tool.get_angle(robot_pos, ball_pos)

        elif self.robot.state == 'aiming' and dist_to_ball > BALL_NEAR_THRESHOLD:
            # ボールが遠い場合、シュートの準備を解除
            self.robot.ball_far()
            
        elif self.robot.state == 'aiming' and shoot_angle < SHOOT_ANGLE_THRESHOLD:
            # シュートの角度が適切な場合、shooting状態に遷移
            self.robot.shoot()
        
        elif self.robot.state == 'shooting' and dist_to_ball <= BALL_NEAR_THRESHOLD:
            # シュートの角度が適切な場合、シュートを実行
            self.move_pos.theta = shoot_angle
            command.kick_power = 10.0
            
        elif self.robot.state == 'shooting':
            # シュートが完了した場合、aiming状態に戻る
            self.robot.done_shooting()
            command.kick_power = 0.0

        command.desired_pose.x = self.move_pos.x
        command.desired_pose.y = self.move_pos.y
        command.desired_pose.theta = self.move_pos.theta

        return command
    
