cmake_minimum_required(VERSION 2.8.3)
project(eed_display)

add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)

find_package(catkin REQUIRED COMPONENTS
        OpenCV REQUIRED
        roscpp
        cv_bridge
        message_filters
        sensor_msgs
        image_transport)

catkin_package(
)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}

)

add_executable(display src/display.cpp)
target_link_libraries(display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

