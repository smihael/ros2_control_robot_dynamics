cmake_minimum_required(VERSION 3.8)
project(ros2_control_robot_dynamics)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-return-type)
  add_compile_options(-O3 -DNDEBUG -march=native)
  add_compile_options(-Wno-maybe-uninitialized -Wno-uninitialized)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(robot_model_impl SHARED
  src/robot_description_loader.cpp
  src/robot_model.cpp
)

target_include_directories(robot_model_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robot_model_impl
  pinocchio
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS robot_model_impl
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(robot_model_impl)
ament_export_dependencies(pinocchio rclcpp rclcpp_lifecycle)

ament_package()
