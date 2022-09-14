# tests for regulated PP
ament_add_gtest(test_pure_pursuit_controller
  test_pure_pursuit_controller.cpp
)
ament_target_dependencies(test_pure_pursuit_controller
  ${dependencies}
)
target_link_libraries(test_pure_pursuit_controller
  ${library_name}
)
