cmake_minimum_required(VERSION 3.0)
project(singletonDtkErrorManager)

set(CMAKE_CXX_STANDARD 11)

find_package(CGAL)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)

set(DTK_INCLUDE_DIR D:\\PROJECTS\\OpenSourceSummer2021\\dtk\\src)
#set(DTK_LIBRARIES_DIR D:\\PROJECTS\\OpenSourceSummer2021\\dtk\\install\\lib)

set(EIGEN_PATH D:\\Eigen\\eigen-3.3.8)

#include_directories(${Boost_INCLUDE_DIR})
#link_directories(${Boost_LIBRARIES_DIR}) 
#include_directories(${DTK_INCLUDE_DIR})
#link_directories(${DTK_LIBRARIES_DIR}) 

include_directories(${EIGEN_PATH})

add_executable(eigenTest main.cpp)

#target_link_libraries(singletonDtkErrorManager ${Boost_LIBRARIES})
#target_link_libraries(singletonDtkErrorManager CGAL)
#target_link_libraries(singletonDtkErrorManager DTK)