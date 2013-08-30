cmake_minimum_required(VERSION 2.8)

project( RAPID C CXX)

find_package( OpenCV REQUIRED )

set( CORE_SOURCES Model.cpp RAPIDTracker.cpp VideoInfo.cpp )

add_executable( RAPID
				${CORE_SOURCES}
				RAPID.cpp )
target_link_libraries( RAPID ${OpenCV_LIBS} )

add_executable( RAPIDTesting
				${CORE_SOURCES}
				RapidTesting.cpp)
target_link_libraries( RAPIDTesting ${OpenCV_LIBS} )