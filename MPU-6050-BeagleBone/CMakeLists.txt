#Generated by VisualGDB project wizard.
#Note: VisualGDB will automatically update this file when you add new sources to the project.

cmake_minimum_required(VERSION 2.7)
project(BBBMPU6050)
set(LIBRARIES_FROM_REFERENCES "")
add_executable(BBBMPU6050 BBBMPU6050.cpp DataRef.cpp Digital.cpp DMP-processing.cpp I2cPort.cpp Ihandler.cpp MPU6050.cpp)
target_link_libraries(BBBMPU6050 "${LIBRARIES_FROM_REFERENCES}")
