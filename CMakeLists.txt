cmake_minimum_required(VERSION 3.0)

project(dtk)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE STRING "Installation directory of library files")

#Installation directory of library files
set(DTK_INSTALL_LIBDIR "${CMAKE_INSTALL_LIBDIR}")

#Installation directory for CMake configuration files
set(DTK_INSTALL_CMAKEDIR "${CMAKE_BINARY_DIR}")

#Installation directory for header files
set(DTK_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")

if("${DTK_INSTALL_LIBDIR}" STREQUAL "")
    set(DTK_INSTALL_LIBDIR "lib")
endif()

if("${DTK_INSTALL_CMAKEDIR}" STREQUAL "")
    set(DTK_INSTALL_CMAKEDIR "bin")
endif()

if("${DTK_INSTALL_INCLUDEDIR}" STREQUAL "")
    set(DTK_INSTALL_INCLUDEDIR "include")
endif()

set(DTK_INSTALL_INCLUDEDIR "include")

message("DTK_INSTALL_LIBDIR : ${DTK_INSTALL_LIBDIR}")
message("DTK_INSTALL_INCLUDEDIR : ${DTK_INSTALL_INCLUDEDIR}")
message("DTK_INSTALL_CMAKEDIR: ${DTK_INSTALL_CMAKEDIR}")

add_subdirectory(src)
