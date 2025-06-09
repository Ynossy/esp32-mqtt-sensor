cmake_minimum_required(VERSION 3.16.0)
list(
    APPEND EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/esp-idf-lib/components" 
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/task-framework"
    )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(HomeAssistantSensors)
