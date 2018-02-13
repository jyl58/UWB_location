project(UWB_Local)

cmake_minimum_required(VERSION 3.0)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED)

include_directories(
inc/
lib/
${BOOST_INCLUDE_DIRS})

ADD_EXECUTABLE(UWB_Local 
src/Interface_UART.cpp
src/uwb.cpp
src/main.cpp) 

target_link_libraries(UWB_Local 
${PROJECT_SOURCE_DIR}/lib/libUWBX.so
${BOOST_LIBRARIES})