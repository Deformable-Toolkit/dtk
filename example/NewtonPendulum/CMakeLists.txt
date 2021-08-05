cmake_minimum_required(VERSION 3.0)
project(NewtonPendulum)
set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(GLFW_INCLUDE_DIR D:\\glfw-3.3.4\\include)
set(GLFW_LIBRARIES_DIR D:\\glfw-3.3.4\\lib-vc2019)

set(GLAD_INCLUDE_DIR D:\\glad\\include)

set(GLM_INCLUDE_DIR D:\\glm-0.9.9.8\\glm)

include_directories(${GLFW_INCLUDE_DIR})
include_directories(${GLAD_INCLUDE_DIR})
include_directories(${GLM_INCLUDE_DIR})
link_directories(${GLFW_LIBRARIES_DIR}) 

#link

#single lib
#glfw static library include  glfw3.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

#multi-thread lib
#glfw static library include  glfw3_mt.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll


add_executable(NewtonPendulum ${dir_source})
target_link_libraries(NewtonPendulum ${GLFW_LIBRARIES} opengl32.lib glfw3dll.lib glfw3.dll)
