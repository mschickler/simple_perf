cmake_minimum_required(VERSION 3.5)

project(simple_perf_test_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(DEPS
  builtin_interfaces
)

foreach(DEP ${DEPS})
    find_package(${DEP} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Probe.msg
  msg/Probe1k.msg
  msg/Probe10k.msg
  msg/Probe100k.msg
  msg/Probe1m.msg
  DEPENDENCIES ${DEPS}
)

ament_package()

