project(UWB_Location)

cmake_minimum_required(VERSION 3.0)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED)

include_directories(
inc/
lib/
${BOOST_INCLUDE_DIRS}
/usr/include/)

ADD_EXECUTABLE(UWB_Location 
src/Interface_UART.cpp
src/uwb.cpp
src/config_parser.cpp
src/main.cpp) 

target_link_libraries(UWB_Location 
${PROJECT_SOURCE_DIR}/lib/libUWBX.so
${BOOST_LIBRARIES}
/usr/lib/arm-linux-gnueabihf/libconfig++.so)