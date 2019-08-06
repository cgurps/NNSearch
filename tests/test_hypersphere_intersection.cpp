#include "../src/KDTree.h"

#define BOOST_TEST_MODULE Queries
#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

typedef boost::mpl::list<float,double> testTypes;

template <std::size_t... Ns>
struct Indices
{
  typedef Indices<Ns...,sizeof...(Ns)> next;
};

template <std::size_t N>
struct MakeIndices
{
  typedef typename MakeIndices<N-1>::type::next type;
};

template<>
struct MakeIndices<0>
{
  typedef Indices<> type;
};

template<typename T, std::size_t N>
bool test()
{
  if(N < 2) return true;

  BoundingBox<T,N> b;
  std::array<T,N> p;

  for(std::size_t dim = 0; dim < N; ++dim)
  {
    b.min[dim] = T(-1.0);
    b.max[dim] = T(1.0);

    p[dim] = T(1.1);
  }

  if(!b.hyperSphereIntersection(p, T(0.1))) 
    return false;
  if(b.hyperSphereIntersection(p, T(0.05))) 
    return false;

  for(std::size_t dim = 0; dim < N; ++dim)
    p[dim] = T(0.0);

  if(!b.hyperSphereIntersection(p, T(0.0))) 
    return false;
  if(!b.hyperSphereIntersection(p, std::numeric_limits<T>::max())) 
    return false;

  return true;
}

#include <initializer_list>

template<typename T, std::size_t... Is>
bool f(Indices<Is...>)
{
  auto list = {(test<T,Is>(), 0)...};
  return ! std::all_of(list.begin(), list.end(), [](bool e){ return e; });
}

BOOST_AUTO_TEST_CASE_TEMPLATE(BoundingBox2D, T, testTypes)
{
  BOOST_REQUIRE(f<T>(MakeIndices<100>::type()));
}
