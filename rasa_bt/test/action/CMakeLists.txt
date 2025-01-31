# Test for the parse action
ament_add_gtest(test_parse_action
  test_parse_action.cpp
)
target_link_libraries(test_parse_action
  rasa_parse_action_bt_node
)
ament_target_dependencies(test_parse_action
  ${dependencies}
)
