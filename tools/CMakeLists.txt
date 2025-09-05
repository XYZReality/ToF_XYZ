cmake_minimum_required(VERSION 3.5)
project(tools)

if (ON_TARGET)
    add_subdirectory(nvm_tools)
endif()

add_subdirectory(software_check)

include(GNUInstallDirs)
