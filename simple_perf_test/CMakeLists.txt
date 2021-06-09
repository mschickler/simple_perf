cmake_minimum_required(VERSION 3.5)
project(simple_perf_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Import ament cmake commands
find_package(ament_cmake REQUIRED)

# List of this packages dependencies
set(DEPS
  rclcpp
  rclcpp_components
  simple_perf_test_msgs
)

# Import packages this package depends on
foreach(DEP ${DEPS})
  find_package(${DEP} REQUIRED)
endforeach()

# Specify source files to compile into package library
add_library(${PROJECT_NAME} SHARED
  src/probe_component.cpp
  src/target_component.cpp
)

# Use header files from this package
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Specify compilation dependencies for package library
ament_target_dependencies(${PROJECT_NAME} ${DEPS})

# Export built library, public headers, and transitive dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})

# Add an executable that combines the two components
add_executable(combined_node src/combined_node.cpp)

target_link_libraries(combined_node ${PROJECT_NAME})

# Install public headers for dependent packages
install(
  DIRECTORY include/
  DESTINATION include
)

### Install libraries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

### Install executables
install(
  TARGETS combined_node
  DESTINATION lib/${PROJECT_NAME}
)

### Install Launch Files ###
#install(FILES
#  launch/my.launch.py
#  DESTINATION share/${PROJECT_NAME}
#)

# Register components and build standalone node executables
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "simple_perf_test::ProbeComponent" EXECUTABLE "probe")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "simple_perf_test::TargetComponent" EXECUTABLE "target")

ament_package()
