#include "KDTree.h"
#include "SplitFunctions.h"

#include <random>
#include <chrono>

int main(int argc, char** argv)
{
  typedef KDPoint<double, 3> Point;

  int nbPoints = 1000000, nbTestPoints = 40000;

  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<double> dist(0.0, 100.0);

  std::vector<std::shared_ptr<const Point>> arr;
  for(int i = 0; i < nbPoints; ++i)
  {
    Point p({dist(rng), dist(rng), dist(rng)});
    arr.push_back(std::make_shared<const Point>(p));
   }

  auto start = std::chrono::high_resolution_clock::now();
  KDTree<double, 3> tree(arr, median<double,3>);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> tTime = finish - start;

  std::cout << "Tree construction time: " << tTime.count() << " s\n";

  tTime = std::chrono::duration<double>(0);
  for(int i = 0; i < nbTestPoints; ++i)
  {
    std::array<double, 3> p = {dist(rng), dist(rng), dist(rng)}; 

    auto start = std::chrono::high_resolution_clock::now();
    Point n = tree.nearest(p);
    auto finish = std::chrono::high_resolution_clock::now();
    tTime += finish - start;
  }

  std::cout << "Query time: " << tTime.count() << " s\n";

  return 0;
}
