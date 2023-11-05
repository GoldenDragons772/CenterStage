cmake_minimum_required(VERSION 3.10)

set(BUILD_SHARED_LIBS ON)
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
        depthai::core
        usb-1.0
        ${log-lib}
)
