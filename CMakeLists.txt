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

add_executable(lantern_output src/lantern_output.cpp)
target_link_libraries(lantern_output ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(recordRouteCommReal src/recordRouteComm_real.cpp)
target_link_libraries(recordRouteCommReal ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
