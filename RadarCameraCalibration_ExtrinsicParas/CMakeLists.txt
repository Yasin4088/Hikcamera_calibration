cmake_minimum_required(VERSION 3.16)
project(radar_camera_extrinsic_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)

set(HIK_MVS_ROOT "$ENV{HIK_MVS_ROOT}" CACHE PATH "Hikvision MVS SDK root path")

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  HINTS
    "${HIK_MVS_ROOT}/include"
    "/opt/MVS/include"
)

find_library(MVS_LIBRARY
  NAMES MvCameraControl
  HINTS
    "${HIK_MVS_ROOT}/lib/64"
    "/opt/MVS/lib/64"
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "Cannot find Hikvision MVS SDK. Set HIK_MVS_ROOT or install SDK under /opt/MVS.")
endif()

add_executable(extrinsic_calibrator
  src/main.cpp
)

target_include_directories(extrinsic_calibrator PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)

target_link_libraries(extrinsic_calibrator PRIVATE
  ${OpenCV_LIBS}
  ${MVS_LIBRARY}
  pthread
)
