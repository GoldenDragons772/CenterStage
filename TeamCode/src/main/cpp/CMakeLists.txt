cmake_minimum_required(VERSION 3.10)

set(CMAKE_DEBUG_POSTFIX "")
set(BUILD_SHARED_LIBS ON)
set(BUILD_TESTS ON)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/opencv/libs/${ANDROID_ABI})

# OpenCV
include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/main/libs/opencv/jni/include
)

add_library(opencv-lib SHARED IMPORTED)
set_target_properties(opencv-lib
        PROPERTIES IMPORTED_LOCATION
        ${LIB_DIR}/libopencv_java4.so
)


# Add depthai-core

add_subdirectory(depthai-core EXCLUDE_FROM_ALL)
project("GDOats")

# add lib-usb
add_subdirectory(libusb)

add_library(
        GDOats
        SHARED
        main.cpp
)



find_library(
        log-lib
        log
)

target_link_libraries(
        GDOats
        opencv-lib
        depthai::opencv
        usb-1.0
        ${log-lib}
)