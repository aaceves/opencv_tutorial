cmake_minimum_required(VERSION 2.8.3)
project(opencv_tutorial)

find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(LIBRARIES OpenCV
               CATKIN_DEPENDS roscpp)

include_directories( ${OpenCV_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS})

add_executable(showGray src/toGray.cpp)
target_link_libraries(showGray ${catkin_LIBRARIES}
                               ${OpenCV_LIBRARIES})

add_executable(showSegmentation src/simpleColorSegmentation.cpp)
target_link_libraries(showSegmentation ${catkin_LIBRARIES}
                                       ${OpenCV_LIBRARIES})

add_executable(showEdges src/detectEdges.cpp)
target_link_libraries(showEdges ${catkin_LIBRARIES}
                                ${OpenCV_LIBRARIES})

add_executable(showDrawings src/drawOnImage.cpp)
target_link_libraries(showDrawings ${catkin_LIBRARIES}
                                   ${OpenCV_LIBRARIES})
