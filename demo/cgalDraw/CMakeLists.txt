cmake_minimum_required(VERSION 3.0)
project(cgalDraw)

set(CMAKE_CXX_STANDARD 11)

SET(CMAKE_PREFIX_PATH C:\\Qt\\5.15.0\\msvc2019_64\\lib/cmake)

SET(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(Qt5Gui REQUIRED)
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(Qt5OpenGL REQUIRED)

find_package(CGAL COMPONENTS Qt5 REQUIRED)
find_package(Qt5 COMPONENTS Core Xml Qml Widgets Gui OpenGL REQUIRED)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)

set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)

set(Qt_INCLUDE_DIR C:\\Qt\\5.15.0\\msvc2019_64\\include)
set(Qt_LIBRARIES_DIR C:\\Qt\\5.15.0\\msvc2019_64\\lib)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARIES_DIR}) 

include_directories(${Qt_INCLUDE_DIR})
link_directories(${Qt_LIBRARIES_DIR}) 

add_executable(cgalDraw ${dir_source})
target_link_libraries(cgalDraw ${Boost_LIBRARIES})
target_link_libraries(cgalDraw CGAL::CGAL CGAL::CGAL_Qt5)
target_link_libraries(cgalDraw Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Qml Qt5::Xml)