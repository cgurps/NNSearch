#include "KDTree.h"

#include <random>
#include <chrono>

int main(int argc, char** argv)
{
  typedef KDPoint<double, 3> Point;

  int nbPoints = 10000, nbTestPoints = 40000;

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
  KDTree<double, 3> tree(arr);
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

    Point nn({0.0, 0.0});
    double minDist = std::numeric_limits<double>::max();
    for(int i = 0; i < nbPoints; ++i)
    {
      if(distance(p, arr[i]->point) < minDist)
      {
        minDist = distance(p, arr[i]->point);
        nn = *(arr[i]);
      }
    }

    if(nn.point != n.point) std::cout << Point(p) << "\n\t" << n << "\n\t" << nn << std::endl;
  }

  std::cout << "Query time: " << tTime.count() << " s\n";

  return 0;
}
