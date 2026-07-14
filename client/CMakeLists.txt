cmake_minimum_required(VERSION 4.1)

project(client)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(client ${SRC_LIST})