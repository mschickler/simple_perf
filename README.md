This code implements the simplest of latency tests between a publisher and a subscriber.

The publisher uses loaned message buffers to take advantage of zero copy shared memory communications with the subscriber that is running in a different process.

Make a workspace directory and clone this repo into it.

Then run "colcon build".

Finally, run the following commands in separate terminal windows from the workspace directory:

RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run simple_perf_test target 

RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=./simple_perf/simple_perf_test/profile/shm_profile.xml ros2 run simple_perf_test probe

