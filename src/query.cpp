#include "KDTree.h"
#include "SplitFunctions.h"
#include "OBJ_Loader.h"
#include "ProgramOptions.h"

#include <random>
#include <chrono>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Point;
typedef KDTree<Scalar, 3>  Tree;

int main(int argc, char** argv)
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<Scalar> dist(-100.0, 100.0);

  ProgramOptions options = parseOptions(argc, argv);

  objl::Loader loader;
  bool loadout = loader.LoadFile(options.filename);
  
  if(loadout)
  {
    objl::Mesh mesh = loader.LoadedMeshes[0];

    std::vector<std::shared_ptr<const Point>> points;
    for(std::size_t i = 0; i < mesh.Vertices.size(); ++i)
    {
      Point p({mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z});
      points.push_back(std::make_shared<const Point>(p));
    }

    Tree tree(points, median<Scalar,3>);
    auto p = tree.nearest(options.point);

    std::cout << p(0) << " " << p(1) << " " << p(2) << std::endl;
  }

  return 0;
}