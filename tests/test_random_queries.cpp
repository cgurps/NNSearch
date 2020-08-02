#include <random>

#include "../src/KDTree.h"
#include "../src/SplitFunctions.h"
#include "../src/OBJ_Loader.h"

class pcg
{
public:
  using result_type = uint32_t;
  static constexpr result_type (min)() { return 0; }
  static constexpr result_type (max)() { return UINT32_MAX; }
  friend bool operator==(pcg const &, pcg const &);
  friend bool operator!=(pcg const &, pcg const &);

  pcg()
      : m_state(0x853c49e6748fea9bULL)
      , m_inc(0xda3e39cb94b95bdbULL)
  {}
  explicit pcg(std::random_device &rd)
  {
      seed(rd);
  }

  void seed(std::random_device &rd)
  {
      uint64_t s0 = uint64_t(rd()) << 31 | uint64_t(rd());
      uint64_t s1 = uint64_t(rd()) << 31 | uint64_t(rd());

      m_state = 0;
      m_inc = (s1 << 1) | 1;
      (void)operator()();
      m_state += s0;
      (void)operator()();
  }

  result_type operator()()
  {
      uint64_t oldstate = m_state;
      m_state = oldstate * 6364136223846793005ULL + m_inc;
      uint32_t xorshifted = uint32_t(((oldstate >> 18u) ^ oldstate) >> 27u);
      int rot = oldstate >> 59u;
      return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
  }

  void discard(unsigned long long n)
  {
      for (unsigned long long i = 0; i < n; ++i)
          operator()();
  }

private:
  uint64_t m_state;
  uint64_t m_inc;
};

bool operator==(pcg const &lhs, pcg const &rhs)
{
  return lhs.m_state == rhs.m_state
    && lhs.m_inc == rhs.m_inc;
}

bool operator!=(pcg const &lhs, pcg const &rhs)
{
  return lhs.m_state != rhs.m_state
    || lhs.m_inc != rhs.m_inc;
}

template <typename T, std::size_t N>
bool test(const std::size_t nbPoints, const std::size_t nbTestPoints, pcg &rng)
{
  typedef Eigen::Matrix<T,N,1> Point;

  std::uniform_real_distribution<T> dist(-100.0, 100.0);

  std::vector<Point*> arr;
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Point *p = new Point(Point::Zero());
    for(std::size_t dim = 0; dim < N; ++dim)
      p->operator()(dim) = dist(rng);
    arr.push_back(p);
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
  pcg rng(dev);

  bool r = test<T,2>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(Queries3D, T, testTypes)
{
  std::size_t nbPoints = 5000, nbTestPoints = 2000;

  std::random_device dev;
  pcg rng(dev);

  bool r = test<T,3>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(Queries20D, T, testTypes)
{
  std::size_t nbPoints = 5000, nbTestPoints = 2000;

  std::random_device dev;
  pcg rng(dev);

  bool r = test<T,20>(nbPoints, nbTestPoints, rng);
  BOOST_CHECK_EQUAL(r, true);
}
