cmake_minimum_required(VERSION 3.0)

project(venombot)

set (CMAKE_CXX_STANDARD 11)

include_directories(include
                    dynamixelSDK/include/dynamixel_sdk)

set(SOURCES src/main.cpp
            src/hexapod.cpp
            src/joystick.cpp
            src/leg.cpp
            src/movement.cpp
            src/motor.cpp
            src/trajectory.cpp
)

set(dxlSOURCES  dynamixelSDK/src/dynamixel_sdk/group_bulk_read.cpp 
                dynamixelSDK/src/dynamixel_sdk/group_bulk_write.cpp 
                dynamixelSDK/src/dynamixel_sdk/group_sync_read.cpp 
                dynamixelSDK/src/dynamixel_sdk/group_sync_write.cpp 
                dynamixelSDK/src/dynamixel_sdk/packet_handler.cpp 
                dynamixelSDK/src/dynamixel_sdk/port_handler.cpp 
                dynamixelSDK/src/dynamixel_sdk/protocol1_packet_handler.cpp 
                dynamixelSDK/src/dynamixel_sdk/protocol2_packet_handler.cpp 
                dynamixelSDK/src/dynamixel_sdk/port_handler_linux.cpp )


add_executable(venombot ${SOURCES} ${dxlSOURCES})

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

target_link_libraries(venombot PRIVATE ${WIRINGPI_LIBRARIES} rt crypt)

