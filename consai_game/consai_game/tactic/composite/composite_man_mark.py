from typing import List, Dict, Set
from dataclasses import dataclass
from consai_msgs.msg import MotionCommand
from consai_game.world_model.world_model import WorldModel
from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.man_mark import ManMark
from consai_game.world_model.threats_model import Threat


@dataclass
class MarkPair:
    assigned_id: int  # マンマークを担当しているロボットID
    target_id: int  # マンマークの対象ロボットID


class ManMarkAssignment:
    def __init__(self):
        self.current_mark_pairs: List[MarkPair] = []
        self.assigned_robots: Set[int] = set()  # マンマークを担当しているロボット
        self.score_threshold = 50  # マンマークを担当するための脅威度の閾値
        self.danger_score_threshold = 65  # 危険な敵ロボットを探すための脅威度の閾値
        self.update_counter = 0  # 内部用更新カウンター

    def register_robot(self, robot_id: int):
        """
        ロボットを登録する関数

        Args:
            robot_id: 登録するロボットID
        """
        if robot_id not in self.assigned_robots:
            self.assigned_robots.add(robot_id)

    def unregister_robot(self, robot_id: int):
        """
        ロボットを登録解除する関数

        Args:
            robot_id: 登録解除するロボットID
        """
        if robot_id in self.assigned_robots:
            self.assigned_robots.discard(robot_id)

    def assign_targets(self, threats: List[Threat]) -> Dict[int, int]:
        """
        各ロボットにマーク対象を割り当てる関数

        Args:
            threats: 脅威度が計算された敵ロボットのリスト

        Returns:
            Dict[int, int]: ロボットIDとマーク対象IDのマッピング
        """
        # danger_score_thresholdを超える敵ロボットで未割り当てのものを探す
        danger_threats = [t for t in threats if t.score >= self.danger_score_threshold]
        assigned_targets = set(pair.target_id for pair in self.current_mark_pairs)
        unassigned_danger_threats = [t for t in danger_threats if t.robot_id not in assigned_targets]

        # unassigned_danger_threatsがあれば、current_mark_pairsの中で一番スコアが低いものを削除
        if unassigned_danger_threats and self.current_mark_pairs:
            # current_mark_pairsの中で一番スコアが低いものを探す
            min_score = float("inf")
            min_pair_index = None
            for i, pair in enumerate(self.current_mark_pairs):
                # 対象のスコアを取得
                score = next((t.score for t in threats if t.robot_id == pair.target_id), float("-inf"))
                if score < min_score:
                    min_score = score
                    min_pair_index = i
            if min_pair_index is not None:
                self.current_mark_pairs.pop(min_pair_index)

        # 既に割り当てられたターゲットを記録する集合
        used_targets = set()
        for pair in list(self.current_mark_pairs):
            # 重複しているターゲットがあれば削除する
            if pair.target_id in used_targets:
                self.current_mark_pairs.remove(pair)
            else:
                used_targets.add(pair.target_id)

        # 新しい割り当て結果を格納するリスト
        new_mark_pairs = []

        # 各担当ロボットに対して処理
        for our_id in self.assigned_robots:
            # 現在のマーク対象を取得
            current_pair = next((pair for pair in self.current_mark_pairs if pair.assigned_id == our_id), None)
            target_id = current_pair.target_id if current_pair else None

            # 現在のマーク対象が依然として脅威（スコア >= score_threshold）なら、そのまま継続
            if target_id is not None and any(
                t.robot_id == target_id and t.score >= self.score_threshold for t in threats
            ):
                new_mark_pairs.append(MarkPair(assigned_id=our_id, target_id=target_id))
                continue

            # 新しいマーク対象を探す
            for threat in threats:
                # 既に割り当て済み、または脅威度が低い（スコア < score_threshold）ならスキップ
                if threat.robot_id in used_targets or threat.score < self.score_threshold:
                    continue
                # 新しいマーク対象を割り当て
                new_mark_pairs.append(MarkPair(assigned_id=our_id, target_id=threat.robot_id))
                used_targets.add(threat.robot_id)
                break

        self.current_mark_pairs = new_mark_pairs
        return {pair.assigned_id: pair.target_id for pair in new_mark_pairs}


class CompositeManMark(TacticBase):
    assignment_module = ManMarkAssignment()

    def __init__(self, default_tactic: TacticBase):
        super().__init__()
        self.default_tactic = default_tactic
        self.man_mark_tactic = ManMark(-1)
        self.mark_target_id = -1

    def reset(self, robot_id: int):
        super().reset(robot_id)

        self.default_tactic.reset(robot_id)
        self.man_mark_tactic.reset(robot_id)

    def exit(self):
        super().exit()
        self.assignment_module.unregister_robot(self.robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        threats = world_model.threats.threats
        # マンマーク役であることを自己申告
        self.assignment_module.register_robot(self.robot_id)

        # 更新カウンターが変わったタイミングで一度だけ実行
        if self.assignment_module.update_counter != world_model.meta.update_counter:
            self.assignment_module.assign_targets(threats)
            self.assignment_module.update_counter = world_model.meta.update_counter
        # 自分の担当するターゲットを取得
        current_pair = next(
            (pair for pair in self.assignment_module.current_mark_pairs if pair.assigned_id == self.robot_id), None
        )
        new_target_id = current_pair.target_id if current_pair else None
        if new_target_id is None:
            # 担当がない場合はデフォルトのtacticを実行
            return self.default_tactic.run(world_model)
        if new_target_id is not None and new_target_id != self.mark_target_id:
            # ターゲットが変わったら更新
            self.mark_target_id = new_target_id
            self.man_mark_tactic.target_robot_id = new_target_id

        return self.man_mark_tactic.run(world_model)
