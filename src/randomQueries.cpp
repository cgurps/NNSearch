/**
 * @file randomQueries.cpp
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 */

/**
 * @dir src
 * @brief The k-d tree header (with the implementation)
 */

#include "KDTree.h"
#include "SplitFunctions.h"
#include "OBJ_Loader.h"
#include "ProgramOptions.h"

#include <random>
#include <chrono>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Point;
typedef KDTree<Scalar, 3>  Tree;

/**
* The main function takes as input a filename for the OBJ
* and a number of test points n. It then performs n random queries
* against the loaded OBJ. The function outputs the tree construction
* time, the memory usage and the overall queries time.
*/
int main(int argc, char** argv)
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<Scalar> dist(-100.0, 100.0);

  ProgramOptions options = parseOptions(argc, argv);

  std::cout << "Loading " << options.filename << "...";
  objl::Loader loader;
  bool loadout = loader.LoadFile(options.filename);
  
  if(loadout)
  {
    std::cout << "Loading successful" << std::endl;
    objl::Mesh mesh = loader.LoadedMeshes[0];

    std::cout << "Building KD-Tree with " << mesh.Vertices.size() << " points..." << std::endl;
    std::vector<std::shared_ptr<const Point>> points;
    for(std::size_t i = 0; i < mesh.Vertices.size(); ++i)
    {
      Point p({mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z});
      points.push_back(std::make_shared<const Point>(p));
    }

    auto start = std::chrono::high_resolution_clock::now();
    Tree tree(points, median<Scalar,3>);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<Scalar> tTime = finish - start;
    std::cout << "Tree construction time: " << tTime.count() * 1000.0 << "ms\n";
    std::cout << "Size of the tree: " << static_cast<double>(tree.memoryUsage()) / (1024.0 * 1024.0) << "mb" << std::endl;
    std::cout << "Size of the points: " << static_cast<double>(points.size() * sizeof(Point)) / (1024.0 * 1024.0) << "mb" << std::endl;

    std::cout << "Performing " << options.nbTestPoints << " random queries..." << std::endl;

    tTime = std::chrono::duration<Scalar>(0);
    for(std::size_t i = 0; i < options.nbTestPoints; ++i)
    {
      Point p = {dist(rng), dist(rng), dist(rng)}; 

      auto start = std::chrono::high_resolution_clock::now();
      tree.nearest(p);
      auto finish = std::chrono::high_resolution_clock::now();
      tTime += finish - start;

      printf("\r%lu/%lu", i + 1, options.nbTestPoints);
      fflush(stdout);
    }

    std::cout << std::endl << "Total query time: " << tTime.count() << " s\n";
    std::cout << "Mean query time: " << tTime.count() / options.nbTestPoints * 1000.0 << " ms\n";
  }

  return 0;
}
