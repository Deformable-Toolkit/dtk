cmake_minimum_required(VERSION 3.0)
project(gltest)
set(CMAKE_CXX_STANDARD 11)

#aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(GLFW_INCLUDE_DIR D:\\glfw-3.3.4\\include)
set(GLFW_LIBRARIES_DIR D:\\glfw-3.3.4\\lib-vc2019)

set(GLM_INCLUDE_DIR D:\\glm-0.9.9.8\\glm)
#set(GLFW_LIBRARIES_DIR D:\\glfw-3.3.4\\lib-vc2019)

set(GLAD_INCLUDE_DIR D:\\glad\\include)

include_directories(${GLFW_INCLUDE_DIR})
include_directories(${GLAD_INCLUDE_DIR})
include_directories(${GLM_INCLUDE_DIR})
link_directories(${GLFW_LIBRARIES_DIR}) 

#link
#add_executable(drawWindows drawWindows.cpp glad.c)
#target_link_libraries(drawWindows ${GLFW_LIBRARIES} opengl32.lib glfw3.lib glfw3_mt.lib)

# glfw3dll.lib glfw3_mt.lib

add_executable(drawTriangle drawTriangle.cpp glad.c)
target_link_libraries(drawTriangle ${GLFW_LIBRARIES} opengl32.lib glfw3dll.lib glfw3.dll)

#add_executable(drawCube drawCube.cpp glad.c stb_image.cpp)
#target_link_libraries(drawCube ${GLFW_LIBRARIES} opengl32.lib glfw3dll.lib glfw3.dll)
