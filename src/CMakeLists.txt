cmake_minimum_required(VERSION 2.8)

project( RAPID C CXX)

find_package( OpenCV REQUIRED )

add_library( CORE_LIB Model.cpp RAPIDTracker.cpp VideoInfo.cpp )

add_executable( RAPID
				RAPID.cpp )
target_link_libraries( RAPID CORE_LIB ${OpenCV_LIBS} )

add_executable( RAPIDTesting
				RapidTesting.cpp)
target_link_libraries( RAPIDTesting CORE_LIB ${OpenCV_LIBS} )