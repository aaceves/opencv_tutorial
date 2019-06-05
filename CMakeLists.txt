cmake_minimum_required(VERSION 2.8.3)
project(opencv_tutorial)

find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(LIBRARIES OpenCV
               CATKIN_DEPENDS roscpp)

include_directories( ${OpenCV_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS})

add_executable(showimage src/toGray.cpp)
target_link_libraries(showimage ${catkin_LIBRARIES}
                                ${OpenCV_LIBRARIES})
