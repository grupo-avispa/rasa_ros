find_package(ament_index_cpp REQUIRED)

# Test for the parse action
ament_add_gtest(test_parse_action test_parse_action.cpp)
target_link_libraries(test_parse_action
  ament_index_cpp::ament_index_cpp
  rasa_parse_action_bt_node
)