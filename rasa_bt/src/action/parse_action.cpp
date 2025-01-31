// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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

#include <string>
#include <memory>

#include "rasa_bt/action/parse_action.hpp"

namespace rasa_bt
{

ParseAction::ParseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<rasa_msgs::action::Parse>(xml_tag_name, action_name, conf)
{
}

void ParseAction::on_tick()
{
  std::string text;
  getInput("text", text);
  std::string message_id;
  getInput("message_id", message_id);

  goal_.text = text;
  goal_.message_id = message_id;
}

BT::NodeStatus ParseAction::on_success()
{
  setOutput("entities", result_.result->entities);
  setOutput("intent", result_.result->intent);
  setOutput("intent_ranking", result_.result->intent_ranking);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rasa_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rasa_bt::ParseAction>(name, "parse", config);
    };

  factory.registerBuilder<rasa_bt::ParseAction>("Parse", builder);
}
