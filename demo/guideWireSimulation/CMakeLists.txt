cmake_minimum_required(VERSION 3.0)
project(guideWireSimulation)

set(Qt5_DIR "C:\\Qt\\5.15.0\\msvc2019_64\\lib\\cmake\\Qt5")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(Qt5 COMPONENTS Core Gui Qml Quick Widgets REQUIRED)
#FIND_PACKAGE(Qt5 COMPONENTS Widgets REQUIRED)
find_package(CGAL)
find_package(Boost)

#Boost
set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)

#dtk
set(DTK_INCLUDE_DIR D:\\PROJECTS\\SimplerRenderer\\dtk\\install\\include)
set(DTK_LIBRARIES_DIR D:\\PROJECTS\\SimplerRenderer\\dtk\\install\\lib)

#freeglut
set(FREEGLUT_INCLUDE_DIR D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\include)
set(FREEGLUT_LIBRARIES_DIR D:\\PROJECTS\\SimplerRenderer\\freeglut-3.2.1\\build\\lib)

#gmp
set(GMP_INCLUDE_DIR D:\\CGAL\\CGAL-4.14.3\\auxiliary\\gmp\\include)
set(GMP_LIBRARIES_DIR D:\\CGAL\\CGAL-4.14.3\\auxiliary\\gmp\\lib)

aux_source_directory(. SOURCE_FILES)
file(GLOB HEADER_FILES *.h)

set(GUIDEWIRE_HEADERS 
	glDrawer.h
	glThread.h
	guideWireSimulation.h
)

set(GUIDEWIRE_FORMS 
	simulationParameters.ui
	guideWireSimulation.ui
)

set(GUIDEWIRE_RESOURCES 
	guideWireSimulation.qrc
)

#调用预编译器moc，需要使用 QT5_WRAP_CPP宏
QT5_WRAP_CPP(GUIDEWIRE_MOC_HEADERS ${GUIDEWIRE_HEADERS})

#使用uic处理.ui文件
QT5_WRAP_UI(GUIDEWIRE_FORMS_HEADERS ${GUIDEWIRE_FORMS})

#使用rcc处理.qrc文件
QT5_ADD_RESOURCES(GUIDEWIRE_RCC_SOURCES ${GUIDEWIRE_RESOURCES})

#这些生成的中间文件都会在build目录下，这样的话，编译器则不能定位由uic程序产生的诸如_ui_mainwindow.h等文件。所以，我们需要把build目录添加到包含目录中
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${DTK_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${FREEGLUT_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GMP_INCLUDE_DIR})

LINK_DIRECTORIES(${Boost_LIBRARIES_DIR})
LINK_DIRECTORIES(${DTK_LIBRARIES_DIR})
LINK_DIRECTORIES(${FREEGLUT_LIBRARIES_DIR})
LINK_DIRECTORIES(${GMP_LIBRARIES_DIR})


#生成可执行文件
ADD_EXECUTABLE(GUIDEWIRE 
	${GUIDEWIRE_SOURCES} 
	${GUIDEWIRE_MOC_HEADERS} 
    	${GUIDEWIRE_FORMS_HEADERS}
	${GUIDEWIRE_RCC_SOURCES} 
	${SOURCE_FILES}
	${HEADER_FILES}
)

#为target添加需要链接的共享库
TARGET_LINK_LIBRARIES(GUIDEWIRE Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick ${Qt5Widgets_LIBRARIES})
#TARGET_LINK_LIBRARIES(GUIDEWIRE ${Qt5Widgets_LIBRARIES})
TARGET_LINK_LIBRARIES(GUIDEWIRE CGAL::CGAL)
TARGET_LINK_LIBRARIES(GUIDEWIRE dtk)
TARGET_LINK_LIBRARIES(GUIDEWIRE ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(GUIDEWIRE ${FREEGLUT_LIBRARIES})
TARGET_LINK_LIBRARIES(GUIDEWIRE libgmp-10.lib libgmp-10.dll)
TARGET_LINK_LIBRARIES(GUIDEWIRE libmpfr-4.lib libmpfr-4.dll)

INSTALL(TARGETS GUIDEWIRE
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
