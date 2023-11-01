cmake_minimum_required(VERSION 3.10)

add_library(Oak SHARED
    src/main/cpp/main.cpp
)

find_library(
        log-lib
        log
)

target_link_libraries(Oak
        ${log-lib}
)