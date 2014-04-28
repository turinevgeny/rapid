cmake_minimum_required(VERSION 2.8)

project( RAPID C CXX)

find_package( OpenCV REQUIRED )

################################
# Core files
################################
set( CORE_HEADERS_LIB
	Ransac.hpp
	Model.hpp
	RAPIDTracker.hpp
	RAPIDTrackerExperiment.hpp
	VideoInfo.hpp
	VideoInfoBase.hpp
	Util.hpp
	Tracker.hpp
	RansacTracker.hpp )
add_library( CORE_LIB
	${CORE_HEADERS_LIB}
	Model.cpp
	RAPIDTracker.cpp
	RAPIDTrackerExperiment.cpp
	VideoInfo.cpp
	Tracker.cpp 
	RansacTracker.cpp )

################################
# RAPID
################################
add_executable( RAPID
				RAPID.cpp )
target_link_libraries( RAPID CORE_LIB ${OpenCV_LIBS} )

################################
# RAPID testing
################################
add_executable( RAPIDTesting
				RapidTesting.cpp)
target_link_libraries( RAPIDTesting CORE_LIB ${OpenCV_LIBS} )

################################
# GTest
################################
set (gtest_SOURCE_DIR ../gtest-1.7.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_library(gtest ${gtest_SOURCE_DIR}/src/gtest-all.cc)
add_library(gtest_main ${gtest_SOURCE_DIR}/src/gtest_main.cc)

find_package(Threads REQUIRED)

target_link_libraries(gtest ${CMAKE_THREAD_LIBS_INIT})

################################
# Tests
################################
set( TEST_HEADERS Util.hpp LineFitting.hpp )
add_executable(
	runUnitTests
    UtilTests.cpp
    LineFittingTests.cpp
	${TEST_HEADERS}
)
target_link_libraries( runUnitTests CORE_LIB ${OpenCV_LIBS} )

# For make-based builds defines make target named test.
# For Visual Studio builds defines Visual Studio project named RUN_TESTS.
enable_testing()

#Compile an executable that will run your unit tests and link it with gtest and gtest_main:
target_link_libraries(runUnitTests gtest gtest_main CORE_LIB ${OpenCV_LIBS})

#Add a test which runs this executable (e.g., make test instruction)
add_test(
    NAME runUnitTests
    COMMAND runUnitTests
)
