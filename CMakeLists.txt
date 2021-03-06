cmake_minimum_required(VERSION 3.1)
project(RTES-trackingCar)
include_directories(.)

set(CMAKE_GCC_FLAGS "-Wall -Wextra -Wconversion -pedantic -std=gnu11")

set(THREADS_PREFER_PTHREAD_FLAG ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

find_package(Threads REQUIRED)
find_package(ptask REQUIRED)
find_package(OpenCV REQUIRED)
find_library(wiringPi_LIB wiringPi)

set(ptask_INCLUDE_DIRS "${ptask_DIR}/include")
message("Including dir ${ptask_INCLUDE_DIRS}...")

add_executable(application src/main.cpp src/car_traker.cpp src/camera.cpp src/control_system.cpp src/detection.cpp)

include_directories ("${ptask_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS} )

target_link_libraries(application ${OpenCV_LIBS} ptask Threads::Threads ${wiringPi_LIB} rt m)
