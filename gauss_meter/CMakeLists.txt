# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS /home/sohamc1909/esp/utils/esp32-arduino-lib-builder/components/arduino_tinyusb)

set(LIBRARY_SRCS 
    /home/sohamc1909/esp/esp-idf-v5-1-3/components/arduino-esp32/libraries/Wire/src/Wire.cpp
    /home/sohamc1909/esp/esp-idf-v5-1-3/components/arduino-esp32/libraries/LiquidCrystal_I2C/LiquidCrystal_I2C.cpp)

set(includedirs 
    /home/sohamc1909/esp/esp-idf-v5-1-3/components/arduino-esp32/libraries/Wire/src/
    /home/sohamc1909/esp/esp-idf-v5-1-3/components/arduino-esp32/libraries/LiquidCrystal_I2C)

project(gauss_meter)

