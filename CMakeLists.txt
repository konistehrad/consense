# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.11)
set(EXTRA_COMPONENT_DIRS ./components/esp-idf-lib/components)
set(COMPONENTS esptool_py esp32 esp32_hap bmp280 main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(consense)
