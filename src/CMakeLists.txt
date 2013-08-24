cmake_minimum_required(VERSION 2.8)

project( RAPID C CXX)

find_package( OpenCV REQUIRED )

set( CORE_LIB Model.cpp RAPIDTracker.cpp VideoInfo.cpp )

add_executable( RAPID
				${CORE_LIB}
				RAPID.cpp )
target_link_libraries( RAPID ${OpenCV_LIBS} )

add_executable( RAPIDTesting
				${CORE_LIB}
				RapidTesting.cpp)
target_link_libraries( RAPIDTesting ${OpenCV_LIBS} )