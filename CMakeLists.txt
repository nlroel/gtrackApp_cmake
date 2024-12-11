cmake_minimum_required(VERSION 3.20)
project(gtrackApp C)

set(CMAKE_C_STANDARD 99)

set(CMAKE_SYSTEM_NAME Windows)

# 添加搜索路径
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}
)

# 查找所有源文件
file(GLOB_RECURSE SOURCES "src/*.c")

## 添加可执行文件
#add_executable(gtrackApp ${SOURCES} gtrackApp.c)

add_library(gtrack ${SOURCES})

add_executable(gtrackApp ${SOURCES} gtrackApp.c gtrackAlloc.c)
target_link_libraries(gtrackApp PUBLIC gtrack)
