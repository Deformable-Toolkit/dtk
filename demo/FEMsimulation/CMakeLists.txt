cmake_minimum_required(VERSION 3.0)
project(FemSimulation)
set(CMAKE_CXX_STANDARD 11)


find_package(CGAL)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(GLFW_INCLUDE_DIR D:\\glfw-3.3.4\\include)
set(GLFW_LIBRARIES_DIR D:\\glfw-3.3.4\\lib-vc2019)
set(GLAD_INCLUDE_DIR D:\\glad\\include)
set(GLM_INCLUDE_DIR D:\\glm-0.9.9.8\\glm)
set(freeglut_include_dir D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\include)
set(freeglut_libraries_dir D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\build\\lib)
set(EIGEN_PATH D:\\Eigen\\eigen-3.3.8)
set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)
set(DTK_INCLUDE_DIR D:\\PROJECTS\\OpenSourceSummer2021\\dtk\\install\\include)
set(DTK_LIBRARIES_DIR D:\\PROJECTS\\OpenSourceSummer2021\\dtk\\install\\lib)

#include_directories(${GLFW_INCLUDE_DIR})
#include_directories(${GLAD_INCLUDE_DIR})
include_directories(${GLM_INCLUDE_DIR})
#link_directories(${GLFW_LIBRARIES_DIR}) 
include_directories(${freeglut_include_dir})
link_directories(${freeglut_libraries_dir})
include_directories(${EIGEN_PATH})
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARIES_DIR}) 
#include_directories(${DTK_INCLUDE_DIR})
#link_directories(${DTK_LIBRARIES_DIR}) 

#link

#single lib
#glfw static library include  glfw3.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

#multi-thread lib
#glfw static library include  glfw3_mt.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll


add_executable(FemSimulation ${dir_source})
#target_link_libraries(FemSimulation ${GLFW_LIBRARIES} glfw3dll.lib glfw3.dll ${Boost_LIBRARIES} CGAL)

target_link_libraries(FemSimulation ${GLFW_LIBRARIES}  ${Boost_LIBRARIES} CGAL freeglut opengl32 glu32)
