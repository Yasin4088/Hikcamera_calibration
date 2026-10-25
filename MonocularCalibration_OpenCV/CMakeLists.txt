cmake_minimum_required(VERSION 3.16)
project(monocular_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d highgui)
find_package(Threads REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation")
find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS "${MVS_ROOT}/include")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_mvs_lib_dir "${MVS_ROOT}/lib/64")
else()
    set(_mvs_lib_dir "${MVS_ROOT}/lib/32")
endif()
find_library(MVS_LIBRARY MvCameraControl HINTS "${_mvs_lib_dir}")
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR "Camera acquisition requires MVS headers/library; set MVS_ROOT")
endif()

add_library(dart_config src/common/DartConfig.cpp)
target_include_directories(dart_config PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(dart_config PUBLIC ${OpenCV_LIBS})
target_compile_options(dart_config PRIVATE -Wall -Wextra -Wpedantic)

add_library(dart_acquisition
    src/Acquisition/Quit.cpp
    src/Acquisition/AcquireFrame.cpp
    src/Acquisition/VideoInput.cpp
)
target_include_directories(dart_acquisition PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(dart_acquisition PUBLIC Threads::Threads ${OpenCV_LIBS})
target_compile_options(dart_acquisition PRIVATE -Wall -Wextra -Wpedantic)

add_library(dart_camera src/Acquisition/CameraInput.cpp)
target_include_directories(dart_camera PRIVATE ${MVS_INCLUDE_DIR})
target_link_libraries(dart_camera PUBLIC dart_acquisition PRIVATE ${MVS_LIBRARY})
target_compile_options(dart_camera PRIVATE -Wall -Wextra -Wpedantic)

set(INTERACTIVE_CALIBRATION_SOURCES
    sub_module/opencv_interactive_calibration/calibController.cpp
    sub_module/opencv_interactive_calibration/frameProcessor.cpp
    sub_module/opencv_interactive_calibration/rotationConverters.cpp
)

add_executable(demarcate
    sub_module/Calibration/demarcate.cpp
    ${INTERACTIVE_CALIBRATION_SOURCES}
)
target_include_directories(demarcate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sub_module
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(demarcate PRIVATE
    dart_config
    dart_camera
    ${OpenCV_LIBS}
)
target_compile_definitions(demarcate PRIVATE DART_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(demarcate PRIVATE -Wall -Wextra -Wpedantic)
