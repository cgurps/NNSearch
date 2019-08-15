#include <random>
#include "../src/KDTree.h"
#include "../src/SplitFunctions.h"
#include "../src/OBJ_Loader.h"

bool testOBJ(const std::string &filename, const std::size_t &nbTestPoints, std::mt19937 &rng)
{
  typedef Eigen::Matrix<double,3,1> Point;

  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  objl::Loader loader;
  if(!loader.LoadFile(filename)) 
    throw std::runtime_error("Error Loading the OBJ");

  objl::Mesh mesh = loader.LoadedMeshes[0];

  std::vector<Point*> arr;
  for(std::size_t i = 0; i < mesh.Vertices.size(); ++i)
  {
    Point *p = new Point({mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z});
    arr.push_back(p);
  }

  KDTree<double,3> tree(arr, median<double,3>);
  for(std::size_t i = 0; i < nbTestPoints; ++i)
  {
    Point p;
    for(std::size_t dim = 0; dim < 3; ++dim)
      p(dim) = dist(rng); 

    Point n = tree.nearest(p);

    Point nn;
    double minDist = std::numeric_limits<double>::max();
    for(std::size_t i = 0; i < arr.size(); ++i)
    {
      if((p - *(arr[i])).norm() < minDist)
      {
        minDist = (p - *(arr[i])).norm();
        nn = *(arr[i]);
      }
    }

    if((n - nn).norm() > 1e-10) return false;
  }

  return true;
}

#define BOOST_TEST_MODULE Queries
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(QueriesOBJ)
{
  std::random_device dev;
  std::mt19937 rng(dev());

  bool r = testOBJ("../shapes/bunny.obj", 1000, rng)
        && testOBJ("../shapes/dragon.obj", 100, rng)
        && testOBJ("../shapes/buddha.obj", 100, rng);

  BOOST_REQUIRE(r == true);
}
