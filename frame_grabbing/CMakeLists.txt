cmake_minimum_required(VERSION 2.8)

project( FrameGrabbing )

find_package( OpenCV REQUIRED )
set(SOURCE FrameGrabbing.cpp)

add_executable(FrameGrabbing ${SOURCE})

target_link_libraries( FrameGrabbing ${OpenCV_LIBS} )
