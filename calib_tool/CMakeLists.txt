cmake_minimum_required(VERSION 2.8)

project( Calibrating )

find_package( OpenCV REQUIRED )
set(SOURCE camera_calibration.cpp)

add_executable(Calibrating ${SOURCE})

target_link_libraries( Calibrating ${OpenCV_LIBS} )
