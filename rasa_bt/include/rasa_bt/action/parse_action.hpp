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

#ifndef RASA_BT__ACTION__PARSE_ACTION_HPP_
#define RASA_BT__ACTION__PARSE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rasa_bt/action/bt_types.hpp"
#include "rasa_msgs/action/parse.hpp"
#include "rasa_msgs/msg/entity.hpp"
#include "rasa_msgs/msg/intent.hpp"

namespace rasa_bt
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps rasa_msgs::action::Parse
 */
class ParseAction : public nav2_behavior_tree::BtActionNode<rasa_msgs::action::Parse>
{
public:
  /**
   * @brief A constructor for rasa_bt::ParseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ParseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("text", "Text to parse"),
        BT::InputPort<std::string>("message_id", "Optional ID for message"),
        BT::OutputPort<std::vector<rasa_msgs::msg::Entity>>("entities", "Parsed entities"),
        BT::OutputPort<rasa_msgs::msg::Intent>("intent", "Intent of the text"),
        BT::OutputPort<std::vector<rasa_msgs::msg::Intent>>("intent_ranking", "Ranking of intents"),
      });
  }
};

}  // namespace rasa_bt

#endif  // RASA_BT__ACTION__PARSE_ACTION_HPP_
