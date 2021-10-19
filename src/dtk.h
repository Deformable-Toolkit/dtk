#ifndef DTK_H
#define DTK_H
/*! 
 * \mainpage Deformation Toolkit Documentation
 *
 * \section intro_sec Introduction
 * DTK has three primary functional parts including Finite Elemend Method,
 * Dynamic Mesh, and Mass-Spring & Rigid-body Simulation. 
 * They are all based on the same common elements,
 *
 * As discussed below.
 *
 * \section part1_sec Part 1 Common Element
 * \subsection be 1. Base Elements 
 * dtkObject
 * - common base class, just like the Object in java.
 *
 * dtkTime
 * - to track time
 * 
 * dtkProperty (dtkReadonlyProperty)
 * - an any-type container
 *
 * dtkErrorManager
 * - standard error reporter
 *
 * \subsection beforcg 2. Base Elements For Computer Graphics
 * dtkIDx 
 * - identifier used for vertex, cell, etc.
 * - define struct dtkID2 dtkID3 dtkID4
 *
 * dtkTx
 * - different length vector of any type
 * - define struct dtkT2 dtkt3 dtkt4
 *
 * dtkPoints (dtkPointsVector)
 * - points container, used in many graphical element in DTK, such as Mesh, Graph
 * dtkPointsReader
 * dtkPointsWriter
 * - points I/O from file.
 * 
 * dtkVolumn
 * - dtkBinaryImage3D(not implement)
 * - encapsule image data source
 *
 * dtkGraph
 * dtkGraphicsKernel
 * - 2D/3D graph data fomat
 * dtkGraphicsTools
 * dtkIntersectTest
 * - 2D/3D graph intersect check algorithm
 *
 * dtkStaticTetraMesh
 * - Tetra mesh data structer
 * dtkStaticTetraMeshReader
 * dtkStaticTetraMeshWriter
 * - Terta mesh I/O from file
 * 
 * dtkStaticTriangleMesh
 * - Triangle mesh data structer
 * dtkStaticTriangleMeshReader
 * dtkStaticTriangleMeshWriter
 * - Triabgle mesh I/O from file
 * 
 * dtkStaticMeshEliminator
 * - designed for Mesh eliminator
 * 
 * \subsection aeforcg 3.  Advanced Elements For Computer Graphics
 * dtkCenterLineMeshing(not implement)
 *
 * dtkCVTMeshing(not implement)
 * - an implement of CVT Meshing which can tranform dtkBinaryImage3D
 *   to dtkStaticTetraMesh
 *
 * dtkDelaunay3D(not implement)
 * - an implement of Delaunay algrithm in 3D.
 *
 * dtkPNTriangles(not implement)
 *
 * \section part2_sec Part 2 Finite Element Method
 * FEM can calculate the stress and displace for nodes in your mesh.
 *
 * It includes
 * - dtkFEMSimulator
 *   - dtkDFEMSimulator
 *     - dtkDLFEMSimulator
 *     - dtkDNFEMSimulator
 *     . 
 *   .
 *   - dtkSFEMSimulator
 *     - dtkSLFEMSimulator
 *     - dtkSNFEMSimulator
 *     . 
 *   .
 * .
 * To name them, 
 * - FEM stands for Finite Element Method; 
 * - D stands for Discrete; 
 * - S stands for Stochastic; 
 * - L stands for Linear; 
 * - N stands for Non-linear.
 *
 * For example, dtkDLFEMSimulator means dtk Discrete Linear Simulator.
 *
 * Choose what you need.
 *
 * \section part3_sec Part 3 Dynamic Mesh
 * The Dynamic Mesh is designed for maintaining the quality of the surface meshes
 *  in the dynamic scenes and support the typical query requirements of the virtual
 *  surgery systems.
 *  
 *  The proposed solution bases on loose r-sample theory. It can effectively protect
 *  the surface meshes' quality during the objects deformation, by maintaining the
 *  loose r-sample properties of the surface sampling points set S, the dynamic
 *  Delaunay triangulation of S, and the labels of the Delaunay cells. In addition,
 *  loose r-sample based dynamic mesh allowing users to use point operations instead
 *  of mesh operations. It also delivers an uniform data structure which supports
 *  the typical mesh operations and greatly reduces the difficulty of interactive
 *  algorithm design for virtual surgery systems.
 *
 *  The proposed mesh reconstruction algorithm and loose r-sample based scene 
 *  management schema are the inheritance and development of the theory of restricted
 *  Delaunay triangulation, r-sample based surface reconstruction algorithm and loose
 *  r-sample based surface reconstruction algorithm. The proposed mesh reconstruction
 *  algorithm extends the r-sample based surface reconstruction algorithm to solid
 *  objects, and generates the tetrahedral meshes; the proposed loose r-sample based
 *  scene management schema applies the loose r-sample related theory to keep the 
 *  quality fo the dynamic meshes.
 *
 * \section part4_sec Part 4 Mass-Spring & Rigid-body Simulation 
 *  This part starts from a dtkScene which is designed for controlling multiple physical
 *  simulations. It allows different types of dtkSceneObject to act.
 *  Different objects will be connected by dtkSceneMediator, which will
 *  transport data among these objects.
 *
 *  The DTK Physical Simulation now includes dtkMassSpring and dtkRigidBody,
 *  and there corresponding mediators.
 *  
 *  In addition, this also provides
 *
 *  - dtkSceneDataCollector
 *      - for storing varies datas during the simulation and allowing other parts do 
 *      further computation, such as rendering.
 *      .
 *  .
 *
 *  - dtkSceneDriver 
 *      - as a external participant in the simulation. Thus, you can interact with 
 *      this system by GUI and so on.  
 *      .
 *  .
 */

#include "dtkConfig.h"
#include "dtkAssert.h"
#include "dtkError.h"
#include "dtkErrorManager.h"

#include "dtkUtility.h"
#include "dtkRandom.h"
//#include "dtkVolume.h"

#include "dtkArray.h"

//Common
#ifdef DTK_CUDA
    #include "dtkCUDA.h"
#endif

#include "dtkTx.h"
#include "dtkTxIO.h"
#include "dtkTxOP.h"
#include "dtkMatrix.h"
#include "dtkMatrixOP.h"

#include "dtkPoints.h"
#include "dtkPointsVector.h"

#include "dtkIDTypes.h"
#include "dtkProperty.h"

//Numeric
//#include "dtkSmallMatrix.h"

//Graphics
#include "dtkGraphics.h"

//Triangle Mesh
#include "dtkStaticTriangleMesh.h"
//#include "dtkStaticTriangleMeshQuery.h"

//#include "dtkDynamicTriangleMesh.h"
//#include "dtkPNTriangles.h"

//Tetrahedra Mesh

//Static
#include "dtkStaticTetraMesh.h"
//#include "dtkStaticTetraMeshQuery.h"
//#include "dtkStaticTetraMeshVertexSorter.h"

//image related operations
//#include "dtkBinaryImage3D.h"
//#include "dtkBinaryImage3DAlgos.h"

//meshing
//#include "dtkDelaunay3D.h"
//#include "dtkCVTMeshing.h"
//#include "dtkCVTDensityFunc.h"
//#include "dtkCVTDensityFuncDistance.h"
//
//#include "dtkGenTetraMeshFromPts.h"

//Dynamic Stage & Objects
//#include "dtkDynamicStage.h"
//#include "dtkDynamicObject.h"
//#include "dtkDynamicObjectImpl.h"
//#include "dtkDynamicAABBTreeImpl.h"
//#include "dtkDynamicBallImpl.h"
//#include "dtkDynamicCellLabel.h"
//#include "dtkDynamicBallCellLabel.h"
//#include "dtkDynamicPointProject.h"
//#include "dtkDynamicBallPointProject.h"
//#include "dtkGraph.h"

//CenterLine
//#include "dtkCenterLineMeshing.h"

//Scene
#include "dtkScene.h"
//#include "dtkMassSpringSimulator.h"
//#include "dtkRigidBodySimulator.h"
//#include "dtkMassSpringDataSourceBasic.h"
//#include "dtkMassSpringMediator.h"
//#include "dtkSceneDataCollector.h"
//#include "dtkSceneDriver.h"

//ports relates
//#include "dtkImports.h"
//#include "dtkExports.h"
//
//#include "dtkPointsImporter.h"
//#include "dtkPointsExporter.h"
//#include "dtkStaticTetraMeshImporter.h"
//#include "dtkStaticTetraMeshExporter.h"
//#include "dtkStaticTriangleMeshImporter.h"
//#include "dtkStaticTriangleMeshExporter.h"

//IO
#include "dtkIO.h"

//FEM
//#include "dtkFEMSolver.h"

//SPH
//#include "dtkSPHSolver.h"

#include "dtkInit.h"

#endif //DTK_H
