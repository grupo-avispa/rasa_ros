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

#ifndef RASA_BT__ACTION__BT_TYPES_HPP_
#define RASA_BT__ACTION__BT_TYPES_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"
#include "rasa_msgs/msg/entity.hpp"
#include "rasa_msgs/msg/intent.hpp"

// Allow bi-directional convertion to JSON
BT_JSON_CONVERTER(rasa_msgs::msg::Entity, entity)
{
  add_field("start", &entity.start);
  add_field("end", &entity.end);
  add_field("value", &entity.value);
  add_field("entity", &entity.entity);
  add_field("confidence", &entity.confidence);
}

BT_JSON_CONVERTER(rasa_msgs::msg::Intent, intent)
{
  add_field("confidence", &intent.confidence);
  add_field("name", &intent.name);
}

// Template specialization to converts a string to Goal.
namespace BT
{
template<> inline rasa_msgs::msg::Entity convertFromString(BT::StringView str)
{
  if (StartWith(str, "json:")) {
    str.remove_prefix(5);
    return convertFromJSON<rasa_msgs::msg::Entity>(str);
  }

  rasa_msgs::msg::Entity output;
  if (!str.empty()) {
    // We expect values separated by ;
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      output.start = convertFromString<int>(parts[0]);
      output.end = convertFromString<int>(parts[1]);
      output.value = convertFromString<std::string>(parts[2]);
      output.entity = convertFromString<std::string>(parts[3]);
      output.confidence = convertFromString<float>(parts[4]);
    }
  }
  return output;
}

template<> inline rasa_msgs::msg::Intent convertFromString(BT::StringView str)
{
  if (StartWith(str, "json:")) {
    str.remove_prefix(5);
    return convertFromJSON<rasa_msgs::msg::Intent>(str);
  }

  // We expect real numbers separated by ;
  auto parts = splitString(str, ';');
  if (parts.size() != 2) {
    throw RuntimeError("invalid input)");
  } else {
    rasa_msgs::msg::Intent output;
    output.confidence = convertFromString<float>(parts[0]);
    output.name = convertFromString<std::string>(parts[1]);
    return output;
  }
}
}  // namespace BT

#endif  // RASA_BT__ACTION__BT_TYPES_HPP_
