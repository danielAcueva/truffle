cmake_minimum_required(VERSION 2.8.3)
project(truffle)

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
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs message_runtime)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare C++ executables
add_executable(vision1 src/vision1.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(vision1 ${OpenCV_LIBS})
target_link_libraries(vision1 ${catkin_LIBRARIES})

# Declare a C++ executable (original)
add_executable(controller1 src/controller1.cpp src/globals.cpp src/helper.cpp src/math.cpp src/strategy/strategy.cpp src/offense.cpp src/defense.cpp src/skills.cpp src/quadrant.cpp src/defensive_plays/D_Moving_Screen.cpp src/offensive_plays/O_Avoid_Corner.cpp src/offensive_plays/O_Dribble_Ball.cpp src/offensive_plays/O_Intercept_Ball.cpp src/offensive_plays/O_Intercept_Avoid.cpp)
add_dependencies(controller1 soccerref_generate_messages_cpp)
target_link_libraries(controller1 ${catkin_LIBRARIES})
