cmake_minimum_required(VERSION 3.17)
project(dtk)

set(CMAKE_CXX_STANDARD 11)

include_directories(src)

#set(Boost_DIR "C:\\local\\boost_1_68_0")
#set(CGAL_DIR "C:\\dev\\CGAL-4.14.3")

find_package(Boost)
#include_directories(${Boost_DIRS})

find_package(CGAL)

add_executable(dtk
        src/dtk.cpp
        src/dtk.h
        src/dtkArray.h
        src/dtkAssert.cpp
        src/dtkAssert.h
        src/dtkCollisionDetectBasic.cpp
        src/dtkCollisionDetectBasic.h
        src/dtkCollisionDetectHierarchy.cpp
        src/dtkCollisionDetectHierarchy.h
        src/dtkCollisionDetectHierarchyKDOPS.cpp
        src/dtkCollisionDetectHierarchyKDOPS.h
        src/dtkCollisionDetectNode.cpp
        src/dtkCollisionDetectNode.h
        src/dtkCollisionDetectNodeKDOPS.cpp
        src/dtkCollisionDetectNodeKDOPS.h
        src/dtkCollisionDetectPrimitive.cpp
        src/dtkCollisionDetectPrimitive.h
        src/dtkCollisionDetectStage.cpp
        src/dtkCollisionDetectStage.h
        src/dtkConfig.h
        src/dtkError.h
        src/dtkErrorManager.cpp
        src/dtkErrorManager.h
        src/dtkExports.h
        src/dtkGraphics.h
        src/dtkGraphicsKernel.cpp
        src/dtkGraphicsKernel.h
        src/dtkGraphicsTools.h
        src/dtkIDTypes.h
        src/dtkImports.h
        src/dtkInit.h
        src/dtkIntersectTest.cpp
        src/dtkIntersectTest.h
        src/dtkIO.h
        src/dtkMatrix.h
        src/dtkMatrixAlgorithm.h
        src/dtkMatrixOP.h
        src/dtkPhysCore.cpp
        src/dtkPhysCore.h
        src/dtkPhysKnotPlanner.cpp
        src/dtkPhysKnotPlanner.h
        src/dtkPhysMassPoint.cpp
        src/dtkPhysMassPoint.h
        src/dtkPhysMassSpring.cpp
        src/dtkPhysMassSpring.h
        src/dtkPhysMassSpringCollisionResponse.cpp
        src/dtkPhysMassSpringCollisionResponse.h
        src/dtkPhysMassSpringThread.cpp
        src/dtkPhysMassSpringThread.h
        src/dtkPhysMassSpringThreadCollisionResponse.cpp
        src/dtkPhysMassSpringThreadCollisionResponse.h
        src/dtkPhysParticle.cpp
        src/dtkPhysParticle.h
        src/dtkPhysParticleSystem.cpp
        src/dtkPhysParticleSystem.h
        src/dtkPhysSpring.cpp
        src/dtkPhysSpring.h
        src/dtkPhysTetraMassSpring.cpp
        src/dtkPhysTetraMassSpring.h
        src/dtkPoints.h
        src/dtkPointsReader.cpp
        src/dtkPointsReader.h
        src/dtkPointsVector.h
        src/dtkPointsWriter.cpp
        src/dtkPointsWriter.h
        src/dtkProperty.h
        src/dtkRandom.h
        src/dtkSign.cpp
        src/dtkSign.h
        src/dtkStaticMeshEliminator.cpp
        src/dtkStaticMeshEliminator.h
        src/dtkStaticTetraMesh.cpp
        src/dtkStaticTetraMesh.h
        src/dtkStaticTetraMeshReader.cpp
        src/dtkStaticTetraMeshReader.h
        src/dtkStaticTetraMeshWriter.cpp
        src/dtkStaticTetraMeshWriter.h
        src/dtkStaticTriangleMesh.cpp
        src/dtkStaticTriangleMesh.h
        src/dtkStaticTriangleMeshReader.cpp
        src/dtkStaticTriangleMeshReader.h
        src/dtkStaticTriangleMeshWriter.cpp
        src/dtkStaticTriangleMeshWriter.h
        src/dtkTime.h
        src/dtkTx.h
        src/dtkTxIO.h
        src/dtkTxOP.h
        src/dtkUtility.h)

target_link_libraries(dtk CGAL::CGAL )
