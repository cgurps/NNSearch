#pragma once

#include "KDTree.h"

template <typename T, std::size_t N>
typename KDTree<T,N>::ArrayIter 
median(const typename KDTree<T,N>::ArrayIter &begin, 
    const typename KDTree<T,N>::ArrayIter &end, 
    const std::size_t depth)
{
  const auto size = std::distance(begin, end);
  std::nth_element(begin, begin + size / 2, end, 
      [&depth](const typename KDTree<T,N>::KDPointSPtr &a
             , const typename KDTree<T,N>::KDPointSPtr &b) -> bool
    {
      return a->point[depth % N] < b->point[depth % N];
    });

  return std::next(begin, size / 2);
}