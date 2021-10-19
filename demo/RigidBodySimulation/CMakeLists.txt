cmake_minimum_required(VERSION 3.12)
project(dtkRigidBody)

set(CMAKE_CXX_STANDARD 14)

set(freeglut_include_dir D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\include)
set(freeglut_libraries_dir D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\build\\lib)

include_directories(${freeglut_include_dir})
link_directories(${freeglut_libraries_dir})

add_executable(dtkRigidBody main.cpp cworld.cpp cworld.h cbody.cpp cbody.h cjoint.cpp cjoint.h cpair.cpp cpair.h ctypes.cpp ctypes.h)
target_link_libraries(dtkRigidBody freeglut opengl32 glu32)