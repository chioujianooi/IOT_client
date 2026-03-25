cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(iotServer VERSION 1.0)

find_package(mysocket REQUIRED)

add_executable(client main.cpp)
target_link_libraries(client mylibs::mysocket)


add_custom_command(TARGET client POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:mylibs::mysocket>
        $<TARGET_FILE_DIR:client>
)