# Install script for directory: D:/PROJECTS/OpenSourceSummer2021/dtk/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/PROJECTS/OpenSourceSummer2021/dtk/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/Debug/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/Release/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/MinSizeRel/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/RelWithDebInfo/dtk.lib")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/Debug/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/Release/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/MinSizeRel/dtk.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/PROJECTS/OpenSourceSummer2021/dtk/clang-build/src/RelWithDebInfo/dtk.lib")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtk.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkArray.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkAssert.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectBasic.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectHierarchy.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectHierarchyKDOPS.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectNode.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectNodeKDOPS.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectPrimitive.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkCollisionDetectStage.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkConfig.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkError.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkErrorManager.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkExports.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkGraphics.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkGraphicsKernel.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkGraphicsTools.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkIDTypes.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkIO.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkImports.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkInit.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkIntersectTest.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkJoint.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkMatrix.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkMatrixAlgorithm.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkMatrixOP.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysCore.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysKnotPlanner.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysMassPoint.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysMassSpring.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysMassSpringCollisionResponse.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysMassSpringThread.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysMassSpringThreadCollisionResponse.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysParticle.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysParticleSystem.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysSpring.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPhysTetraMassSpring.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPoints.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPointsReader.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPointsVector.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkPointsWriter.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkProperty.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkRandom.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkRigidBody.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkScene.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkSign.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticMeshEliminator.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTetraMesh.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTetraMeshReader.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTetraMeshWriter.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTriangleMesh.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTriangleMeshReader.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkStaticTriangleMeshWriter.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkTime.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkTx.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkTxIO.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkTxOP.h"
    "D:/PROJECTS/OpenSourceSummer2021/dtk/src/dtkUtility.h"
    )
endif()

