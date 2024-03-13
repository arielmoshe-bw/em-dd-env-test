cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 11)
set(PROJECT_NAME dd-env-test)
set(${PROJECT_NAME}_BOARD mega2560)

project(${PROJECT_NAME} VERSION 1.0.0)

# Define additional source and header files or default arduino sketch files

file(GLOB ${PROJECT_NAME}_SOURCE_FILES
        src/*.*
        src/DirectDriveEnvironmentTestManager/*.*
        src/DirectDriveSteeringWheel/*.*
)

### Additional static libraries to src in the target.
file(GLOB ${PROJECT_NAME}_LIBS
        /usr/share/arduino/hardware/arduino/cores/arduino/*.*
        /usr/share/arduino/libraries/SPI/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/libraries/ArduinoLog/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/libraries/autowp-mcp2515/*.*
)

### Main sketch file
set(${PROJECT_NAME}_SKETCH ${PROJECT_NAME}.ino)

### Additional settings to add non-standard or your own Arduino libraries.
# For this example (libs will contain additional arduino libraries)
# An Arduino library my_lib will contain files in libs/my_lib/: my_lib.h, my_lib.cpp + any other cpp files
include_directories(
        /usr/share/arduino/lib
        /usr/share/arduino/libraries
        /usr/share/arduino/hardware/arduino/cores/arduino
        /usr/share/arduino/hardware/arduino/variants/mega

        /usr/share/arduino/libraries/SPI

        ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/libraries
        ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/libraries/ArduinoLog
        ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/libraries/autowp-mcp2515

        src/
        src/DirectDriveEnvironmentTestManager
        src/DirectDriveSteeringWheel
)

# For nested library sources replace ${LIB_NAME} with library name for each library
# set(_RECURSE true)

add_executable(${PROJECT_NAME}_lib ${PROJECT_NAME}.ino
        ${${PROJECT_NAME}_LIBS}
        ${${PROJECT_NAME}_SOURCE_FILES}
)

#### Additional settings for programmer. From programmers.txt
set(${PROJECT_NAME}_PROGRAMMER avrisp)
set(${PROJECT_NAME}_PORT /dev/ttyACM0)
set(uno.upload.speed 115200)