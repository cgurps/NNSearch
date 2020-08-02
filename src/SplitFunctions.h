#pragma once

/**
 * @file SplitFunctions.h
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 * @brief Splitting functions for the k-d tree construction
 */

/**
 * @dir src
 * @brief The source directory
 */

#include "KDTree.h"

/**
 * Return the median from the input range at a given dimension. I use std::nth_element over std::sort
 * to speed up the median computation (the average complexity goes from O(n log(n))
 * to O(n)).
 * @param begin the iterator to the beginning of the range
 * @param end the iterator to the end of the range
 * @param current depth of the tree
 */
template <typename T, std::size_t N>
typename KDTree<T,N>::ArrayIter
median(const typename KDTree<T,N>::ArrayIter &begin,
    const typename KDTree<T,N>::ArrayIter &end,
    const std::size_t depth)
{
  const auto size = std::distance(begin, end);
  std::nth_element(begin, begin + size / 2, end,
      [&depth](const typename KDTree<T,N>::Point* a
             , const typename KDTree<T,N>::Point* b) -> bool
    {
      return a->operator()(depth % N) < b->operator()(depth % N);
    });

  return std::next(begin, size / 2);
}
