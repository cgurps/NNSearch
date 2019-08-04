#include <random>
#include "../src/KDTree.h"
#include "../src/SplitFunctions.h"

#define ASSERT_THROW( condition )                                   \
{                                                                   \
  if( condition )                                                   \
  {                                                                 \
    throw std::runtime_error(   std::string( __FILE__ )             \
                              + std::string( ":" )                  \
                              + std::to_string( __LINE__ )          \
                              + std::string( " in " )               \
                              + std::string( __PRETTY_FUNCTION__ )  \
    );                                                              \
  }                                                                 \
}

template <typename T, std::size_t N>
void test(const std::size_t nbPoints, const std::size_t nbTestPoints, std::mt19937 &rng)
{
  typedef KDPoint<T,N> Point;

  std::uniform_real_distribution<T> dist(0.0, 100.0);

  std::vector<std::shared_ptr<const Point>> arr;
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Point p;
    for(std::size_t dim = 0; dim < N; ++dim)
      p.point[dim] = dist(rng);;
    arr.push_back(std::make_shared<const Point>(p));
   }

  KDTree<T,N> tree(arr, median<T,N>);
  for(std::size_t i = 0; i < nbTestPoints; ++i)
  {
    std::array<T,N> p;
    for(std::size_t dim = 0; dim < N; ++dim)
      p[dim] = dist(rng); 

    Point n = tree.nearest(p);

    Point nn;
    double minDist = std::numeric_limits<T>::max();
    for(std::size_t i = 0; i < nbPoints; ++i)
    {
      if(distance(p, arr[i]->point) < minDist)
      {
        minDist = distance(p, arr[i]->point);
        nn = *(arr[i]);
      }
    }

    ASSERT_THROW(distance(nn.point, n.point) > T(1e-10));
  }
}
