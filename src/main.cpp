#include "KDTree.h"
#include "SplitFunctions.h"
#include "OBJ_Loader.h"
#include "ProgramOptions.h"

#include <random>
#include <chrono>

int main(int argc, char** argv)
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  ProgramOptions options = parseOptions(argc, argv);

  std::cout << "Loading " << options.filename << "...";
  objl::Loader loader;
  bool loadout = loader.LoadFile(options.filename);
  
  if(loadout)
  {
    std::cout << "Loading successful" << std::endl;
    objl::Mesh mesh = loader.LoadedMeshes[0];

    typedef KDPoint<double, 3> Point;

    std::cout << "Building KD-Tree with " << mesh.Vertices.size() << " points..." << std::endl;
    std::vector<std::shared_ptr<const Point>> points;
    for(std::size_t i = 0; i < mesh.Vertices.size(); ++i)
    {
      Point p({mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z});
      points.push_back(std::make_shared<const Point>(p));
    }

    auto start = std::chrono::high_resolution_clock::now();
    KDTree<double, 3> tree(points, median<double,3>);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tTime = finish - start;
    std::cout << "Tree construction time: " << tTime.count() << " s\n";

    std::cout << "Performing " << options.nbTestPoints << " random queries..." << std::endl;

    tTime = std::chrono::duration<double>(0);
    for(std::size_t i = 0; i < options.nbTestPoints; ++i)
    {
      std::array<double, 3> p = {dist(rng), dist(rng), dist(rng)}; 

      auto start = std::chrono::high_resolution_clock::now();
      Point n = tree.nearest(p);
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
