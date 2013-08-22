cmake_minimum_required(VERSION 2.8)

project( RAPID C CXX)

find_package( OpenCV REQUIRED )

set(SOURCE RAPID.cpp Model.cpp RAPIDTracker.cpp VideoInfo.cpp)
add_executable(RAPID ${SOURCE})

target_link_libraries( RAPID ${OpenCV_LIBS} )
