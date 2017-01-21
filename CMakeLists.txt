cmake_minimum_required(VERSION 2.8.3)
project(truffleshuffle)

set(CMAKE_CXX_STANDARD 11)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    message_generation
    cv_bridge
    genmsg
    image_transport
    sensor_msgs
    xacro
    soccerref
)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs message_runtime soccerref)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(controller1 src/controller1.cpp src/test.cpp src/math.cpp src/strategy.cpp src/offense.cpp src/defense.cpp)
add_dependencies(controller1 soccerref_generate_messages_cpp)
target_link_libraries(controller1 ${catkin_LIBRARIES})