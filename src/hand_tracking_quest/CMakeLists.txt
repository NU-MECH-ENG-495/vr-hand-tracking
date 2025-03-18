cmake_minimum_required(VERSION 3.8)
project(hand_tracking_quest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find Qt packages.
find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(qt_joint_angles_viewer src/JointAngleVisualizer.cpp)
ament_target_dependencies(qt_joint_angles_viewer rclcpp std_msgs)
target_link_libraries(qt_joint_angles_viewer Qt5::Widgets)

add_executable(hand_tracker_quest_node src/HandTrackerQuest.cpp)
ament_target_dependencies(hand_tracker_quest_node rclcpp std_msgs)

# Create a library target for the production code.
add_library(hand_tracker_quest_lib
  src/HandTrackerQuest.cpp
)
target_include_directories(hand_tracker_quest_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hand_tracker_quest_lib rclcpp std_msgs)


target_include_directories(hand_tracker_quest_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  hand_tracker_quest_node
  qt_joint_angles_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_gtest(test_hand_tracker_node test/test_hand_tracker_node.cpp)
  if(TARGET test_hand_tracker_node)
    target_include_directories(test_hand_tracker_node PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_hand_tracker_node
      rclcpp
      std_msgs
    )
  endif()
endif()




# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
