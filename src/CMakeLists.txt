cmake_minimum_required(VERSION 2.8.7)

project( RAPID )

find_package( OpenCV REQUIRED )
set(SOURCE RAPID.cpp Model.cpp RAPIDTracker.cpp)

#SET(SOURCE_LIB Model.cpp)
#add_library(Model STATIC ${SOURCE_LIB})

add_executable(RAPID ${SOURCE})

#target_link_libraries( RAPID Model)
target_link_libraries( RAPID ${OpenCV_LIBS} )
