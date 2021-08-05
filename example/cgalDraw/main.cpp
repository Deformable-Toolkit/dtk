#define CGAL_USE_BASIC_VIEWER
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Sphere_3.h>
#include <fstream>
#include <iostream>

typedef CGAL::Simple_cartesian<double>                       Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef Kernel::Point_3                                      Point;
typedef CGAL::Surface_mesh<Point>                            Mesh;
typedef K::Sphere_3                                     Sphere;

int main(int argc, char* argv[])
{
  Mesh sm1;
  std::ifstream in1((argc>1)?argv[1]:"data/triangle.off");
  in1 >> sm1;

  CGAL::Point_3<K> center(0,0,3);

  Sphere s(center, 1.0);

  CGAL::draw(s);

  CGAL::draw(sm1);

  getchar();

  return EXIT_SUCCESS;
}