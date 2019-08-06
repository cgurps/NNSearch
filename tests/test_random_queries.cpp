#include <random>
#include "../src/KDTree.h"
#include "../src/SplitFunctions.h"
#include "../src/OBJ_Loader.h"

template <typename T, std::size_t N>
bool test(const std::size_t nbPoints, const std::size_t nbTestPoints, std::mt19937 &rng)
{
  typedef Eigen::Matrix<T,N,1> Point;

  std::uniform_real_distribution<T> dist(-100.0, 100.0);

  std::vector<std::shared_ptr<const Point>> arr;
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Point p;
    for(std::size_t dim = 0; dim < N; ++dim)
      p(dim) = dist(rng);;
    arr.push_back(std::make_shared<const Point>(p));
   }

  KDTree<T,N> tree(arr, median<T,N>);
  for(std::size_t i = 0; i < nbTestPoints; ++i)
  {
    Point p;
    for(std::size_t dim = 0; dim < N; ++dim)
      p(dim) = dist(rng); 

    Point n = tree.nearest(p);

    Point nn;
    double minDist = std::numeric_limits<T>::max();
    for(std::size_t i = 0; i < nbPoints; ++i)
    {
      if((p - *(arr[i])).norm() < minDist)
      {
        minDist = (p - *(arr[i])).norm();
        nn = *(arr[i]);
      }
    }

    if((nn - n).norm() > T(1e-10)) return false;
  }

  return true;
}

#define BOOST_TEST_MODULE Queries
#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

typedef boost::mpl::list<float,double> testTypes;

BOOST_AUTO_TEST_CASE_TEMPLATE(Queries2D, T, testTypes)
{
  std::size_t nbPoints = 5000, nbTestPoints = 2000;

  std::random_device dev;
  std::mt19937 rng(dev());

  bool r = test<T,2>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(Queries3D, T, testTypes)
{
  std::size_t nbPoints = 5000, nbTestPoints = 2000;

  std::random_device dev;
  std::mt19937 rng(dev());

  bool r = test<T,3>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(Queries20D, T, testTypes)
{
  std::size_t nbPoints = 5000, nbTestPoints = 2000;

  std::random_device dev;
  std::mt19937 rng(dev());

  bool r = test<T,20>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}
