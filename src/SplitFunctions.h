#pragma once

/**
 * @file SplitFunctions.h
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 */

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
      return a->operator()(depth % N) < b->operator()(depth % N);
    });

  return std::next(begin, size / 2);
}
