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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "rasa_bt/action/parse_action.hpp"

class ParseActionServer
  : public TestActionServer<rasa_msgs::action::Parse>
{
public:
  ParseActionServer()
  : TestActionServer("parse")
  {
  }

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<rasa_msgs::action::Parse>> goal_handle)
  override
  {
    rasa_msgs::action::Parse::Result::SharedPtr result =
      std::make_shared<rasa_msgs::action::Parse::Result>();
    rasa_msgs::msg::Entity entity;
    entity.start = 1;
    entity.end = 2;
    entity.value = "test";
    entity.entity = "test2";
    entity.confidence = 0.5;
    result->entities.push_back(entity);
    rasa_msgs::msg::Intent intent;
    intent.name = "test";
    intent.confidence = 1.5;
    result->intent = intent;
    result->intent_ranking.push_back(intent);
    result->intent_ranking.push_back(intent);

    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class ParseActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("parse_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<rasa_bt::ParseAction>(name, "parse", config);
      };

    factory_->registerBuilder<rasa_bt::ParseAction>("Parse", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ParseActionServer> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ParseActionTestFixture::node_ = nullptr;
std::shared_ptr<ParseActionServer> ParseActionTestFixture::server_ = nullptr;
BT::NodeConfiguration * ParseActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ParseActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ParseActionTestFixture::tree_ = nullptr;

TEST_F(ParseActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Parse/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Parse text="" message_id="" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_TRUE(tree_->rootNode()->getInput<std::string>("text").value().empty());
  EXPECT_TRUE(tree_->rootNode()->getInput<std::string>("message_id").value().empty());

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Parse text="This is a test" message_id="This is also a test" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("text"), "This is a test");
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("message_id"), "This is also a test");
}

TEST_F(ParseActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Parse text="This is a test" entities="{entities}" intent="{intent}" intent_ranking="{intent_ranking}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("text"), "This is a test");

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // Check if the output is correct
  auto entities = config_->blackboard->get<std::vector<rasa_msgs::msg::Entity>>("entities");
  EXPECT_EQ(entities.size(), 1);
  EXPECT_EQ(entities[0].start, 1);
  EXPECT_EQ(entities[0].end, 2);
  EXPECT_EQ(entities[0].value, "test");
  EXPECT_EQ(entities[0].entity, "test2");
  EXPECT_EQ(entities[0].confidence, 0.5);

  auto intent = config_->blackboard->get<rasa_msgs::msg::Intent>("intent");
  EXPECT_EQ(intent.confidence, 1.5);
  EXPECT_EQ(intent.name, "test");

  auto intent_ranking =
    config_->blackboard->get<std::vector<rasa_msgs::msg::Intent>>("intent_ranking");
  EXPECT_EQ(intent_ranking.size(), 2);
  EXPECT_EQ(intent_ranking[0].confidence, 1.5);
  EXPECT_EQ(intent_ranking[0].name, "test");
  EXPECT_EQ(intent_ranking[1].confidence, 1.5);
  EXPECT_EQ(intent_ranking[1].name, "test");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  ParseActionTestFixture::server_ = std::make_shared<ParseActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(ParseActionTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
