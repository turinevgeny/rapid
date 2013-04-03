#${OpenCV_DIR}="/home/user/OpenCV-2.4.3/build"
cmake_minimum_required(VERSION 2.8)
project( rapid )
find_package( OpenCV REQUIRED )
set(Path ${CMAKE_SOURCE_DIR}/src )
set(SOURCE ${Path}/Rapid.cpp ${Path}/Model.cpp)
add_executable( rapid ${SOURCE} )
target_link_libraries( rapid ${OpenCV_LIBS} )
