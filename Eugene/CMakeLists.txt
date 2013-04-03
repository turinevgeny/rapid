cmake_minimum_required(VERSION 2.8)

project( RAPID )

find_package( OpenCV REQUIRED )

SET(SOURCE_EXE RAPID.cpp Model.cpp)
#SET(SOURCE_LIB Model.cpp)

#add_library(Model STATIC ${SOURCE_LIB})
add_executable(RAPID ${SOURCE_EXE})

#target_link_libraries( RAPID Model)
target_link_libraries( RAPID ${OpenCV_LIBS} )
