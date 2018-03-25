cmake_minimum_required(VERSION 2.6)

project(planar)

add_subdirectory(isam)

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()
message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

##########
#include_directories(/usr/include/openni2)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/Include)
##########

set(SRCS src/main.cpp src/planes.cpp src/Logger.cpp src/Mapper.cpp src/Visualizer.cpp)

# C++11
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

##########
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOpenNI2x64.so ${CMAKE_CURRENT_BINARY_DIR}/libOpenNI2.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libDummyDevicex64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libDummyDevice.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOniFilex64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libOniFile.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libPS1080x64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libPS1080.so COPYONLY)

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,. -L${CMAKE_CURRENT_BINARY_DIR}")
##########

# iSAM and Cholmod
include_directories(${CHOLMOD_INCLUDES} isam/include)

add_executable(planar ${SRCS} src/Camera.cpp)
target_link_libraries(planar ${PCL_LIBRARIES})
#target_link_libraries(planar -L/usr/lib/OpenNI2/Drivers -lOpenNI2 -lDummyDevice -lOniFile -lPS1080)
target_link_libraries(planar ${OpenCV_LIBRARIES})
target_link_libraries(planar isamlib -lz)

##########
target_link_libraries(planar OpenNI2)
##########

add_executable(simulate src/simulate.cpp src/Simulation.cpp)
set_source_files_properties(src/Simulation.cpp src/simulate.cpp PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(simulate isamlib -lz)

include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_executable(lcmlisten src/LCMListen.cpp)
set_source_files_properties(src/LCMListen.cpp PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(lcmlisten 
  -lz 
  lcm
  ${OpenCV_LIBRARIES}
)

# unit tests
enable_testing()
add_test(NAME MyTest COMMAND Test)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK)
add_executable(Test src/test.cpp)
target_link_libraries(Test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

