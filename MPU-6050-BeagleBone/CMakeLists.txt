cmake_minimum_required(VERSION 2.8)
project(Linux-I2C)

if (${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

set(MPU6050_SRC_FILES
        MPU6050.cpp
        MPU6050.h
        I2cPort.cpp
        I2cPort.h
        TestMpu6050DMP6.cpp
	helper_3dmath.h
	DMP-processing.h
	DMP-processing.cpp
	DataRef.h
	DataRef.cpp
	Digital.h
	Digital.cpp)
	
add_executable(MPU6050 ${MPU6050_SRC_FILES})
