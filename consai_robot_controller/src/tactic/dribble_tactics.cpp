// Copyright 2024 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "consai_robot_controller/tactic/dribble_tactics.hpp"
#include "consai_tools/geometry_tools.hpp"

#include <iostream>

namespace dribble_tactics
{

namespace tools = geometry_tools;


DribbleTactics::DribbleTactics()
{
  tactic_functions_["APPROACH"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Approaching the ball" << std::endl;
    return "CATCH";
  };

  tactic_functions_["CATCH"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Catching the ball" << std::endl;
    return "CARRY";
  };

  tactic_functions_["CARRY"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Carrying the ball" << std::endl;
    return "RELEASE";
  };

  tactic_functions_["RELEASE"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Releasing the ball" << std::endl;
    return "FINISH";
  };

  tactic_functions_["FINISH"] = [this](TacticDataSet & data_set) -> TacticName {
    std::cout << "Dribbling is finished." << std::endl;
    return "FINISH";
  };
}

bool DribbleTactics::update(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power)
{
  const auto robot_id = my_robot.robot_id.id;
  // If the robot is not in the map, initialize the tactic state.
  if (tactic_name_.find(robot_id) == tactic_name_.end()) {
    tactic_name_[robot_id] = "APPROACH";
  }

  // Execute the tactic fuction.
  TacticDataSet data_set(parsed_pose, dribble_target, my_robot, ball);
  tactic_name_[robot_id] = tactic_functions_[tactic_name_[robot_id]](data_set);

  parsed_pose = data_set.get_parsed_pose();
  parsed_dribble_power = data_set.get_parsed_dribble_power();

  return true;
}

}  // namespace dribble_tactics
