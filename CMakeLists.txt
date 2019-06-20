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

add_executable(spaceColors src/differentColorSegmentation.cpp)
target_link_libraries(spaceColors ${catkin_LIBRARIES}
                                  ${OpenCV_LIBRARIES})

add_executable(showEdges src/detectEdges.cpp)
target_link_libraries(showEdges ${catkin_LIBRARIES}
                                ${OpenCV_LIBRARIES})

add_executable(showDrawings src/drawOnImage.cpp)
target_link_libraries(showDrawings ${catkin_LIBRARIES}
                                   ${OpenCV_LIBRARIES})

add_executable(clickMe src/clickMe.cpp)
target_link_libraries(clickMe ${catkin_LIBRARIES}
                              ${OpenCV_LIBRARIES})

add_executable(showHist src/computeHistogram.cpp)
target_link_libraries(showHist ${catkin_LIBRARIES}
                               ${OpenCV_LIBRARIES})

add_executable(readWriteFile src/readWriteFile.cpp)
target_link_libraries(readWriteFile ${catkin_LIBRARIES}
                                    ${OpenCV_LIBRARIES})

add_executable(showBlobs src/showBlobs.cpp)
target_link_libraries(showBlobs ${catkin_LIBRARIES}
                                ${OpenCV_LIBRARIES})

add_executable(showVideo src/readWriteVideo.cpp)
target_link_libraries(showVideo ${catkin_LIBRARIES}
                                ${OpenCV_LIBRARIES})

add_executable(cropMergeWrite src/cropMergeWrite.cpp)
target_link_libraries(cropMergeWrite ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES})

add_executable(ballTracking src/detectBall.cpp)
target_link_libraries(ballTracking ${catkin_LIBRARIES}
                                   ${OpenCV_LIBRARIES})

add_executable(faceDetectionHaar src/detectFaces.cpp)
target_link_libraries(faceDetectionHaar ${catkin_LIBRARIES}
                                        ${OpenCV_LIBRARIES})
