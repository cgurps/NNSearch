#pragma once

/**
 * @file KDTree.h
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 * @brief The k-d tree header (with the implementation)
 */

/**
 * @dir src
 * @brief The source directory
 */

#include <utility>
#include <array>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <type_traits>
#include <functional>
#include "assert.h"
#include <queue>

#include <Eigen/Dense>

/**
 * @struct BoundingBox
 * @brief A simple bounding box
 * @tparam T the type of point (must be a real floating point type)
 * @tparam N the dimension of the point (must be greater than zero)
 */
template <class T, std::size_t N>
struct BoundingBox
{
  /**
   * Alias for the internal point structure
   */
  typedef typename Eigen::Matrix<T,N,1> Point;

  BoundingBox()
    : min(Point::Zero()), max(Point::Zero())
  {}

  /**
   * Constructor of the bounding box
   * @param min the lower end of the bounding box
   * @param max the upper end of the bounding box
   */
  BoundingBox(const Point &min, const Point &max)
    : min(std::move(min)), max(std::move(max))
  {}

  /**
   * rvalue constructor of the bounding box
   * @param min the lower end of the bounding box
   * @param max the upper end of the bounding box
   */
  BoundingBox(const Point &&min, const Point &&max)
    : min(std::move(min)), max(std::move(max))
  {}

  /**
   *  Check if an hypersphere intersects the bounding box.
   *  The algorithm is an adaptation in N dimension of the method proposed by e.James
   *  from https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection.
   *  The algorithm restricts checks within one corner of the bounding box.
   *  @param p the center of the hypersphere
   *  @param radius the radius of the hypersphere
   */
  bool hyperSphereIntersection(const Point &p, const T &radius) const;

  /**
   * The lower end of the bounding box
   */
  Point min;

  /**
   * The upper end of the bounding box
   */
  Point max;
};

/** @class KDTree
 *  @brief Container for the KD-tree
 *
 *  @tparam T the type of point (must be a real floating point type)
 *  @tparam N the dimension of the point (must be greater than zero)
 */
template <class T, std::size_t N>
class KDTree
{
  public:
    /**
     * Alias for the internal point structure
     */
    typedef Eigen::Matrix<T,N,1> Point;

    /**
     * Alias for the input KDPoint shared pointer vector
     */
    typedef typename std::vector<Point*> KDPointArray;

    /**
     * Alias for the KDPointArray iterator
     */
    typedef typename KDPointArray::iterator ArrayIter;

    /**
     * Alias for the split function used for the construction of the tree
     */
    using SplitFunction = ArrayIter(*)(const ArrayIter&, const ArrayIter&, const std::size_t);

    /**
     * Deletion of the default constructor
     */
    KDTree() = delete;

    /**
     * Deletion of the copy constructor
     */
    KDTree(const KDTree &) = delete;

    /**
     * Deletion of the assignement constructor
     */
    KDTree &operator = (const KDTree &) = delete;

    /**
     * Default destructor
     */
    ~KDTree() = default;

    /**
     * Construct a KDTree from an array of KDPoint
     * @param arr the input array of KDPoint
     * @param splitFun the split function used for the construction
     */
    KDTree(KDPointArray &arr, const SplitFunction &splitFun);

    /**
     * Interface for the nearest point query
     * @param point the input point
     */
    Point nearest(const Point &point) const;

    /**
     * Estimates the current memory usage of the tree
     */
    std::size_t memoryUsage()
    {
      return 3 * N * sizeof(T) * tree.size();
    }

  private:
    /**
     * Recursive function for the KDTree construction.
     * The tree is built by splitting the input range using the Split Function provided by
     * the constructor. It halts when the range is reduced to one point (ie begin = end).
     * @param begin an iterator to the beginning of the array for the current level of recursion
     * @param end and iterator to the end of the array for the current level of recursion
     * @param box the parent bounding box
     * @param depth the current depth
     */
    void makeTree(const ArrayIter &begin,
        const ArrayIter &end,
        const BoundingBox<T,N> box,
        const std::size_t treePos = 0,
        const std::size_t depth = 0);

    /**
     * Recursive function to find the nearest point in the KDTree.
     * The algorithm checks first if the current node is either null or is a leaf and halts if so.
     * Then it also halts if the distance from the query node to the node bounding box is less than the current best distance.
     * Finally, the algorithm chooses either to explore the left or the right of the tree depending on the tree node value
     * at the current dimension (the current dimension is determined by the depth of the current node).
     * @param node the current node
     * @param point the point for the query
     * @param closest the current closest point found
     * @param minDist the current minimum distance found
     * @param depth the current depth of the recursion
     */
    void nearest(const std::size_t node,
        const Point &point,
        Point &closest,
        T &minDist,
        const std::size_t depth = 0) const;

    /**
     * Array representing the tree
     */
    std::vector<std::pair<Point, BoundingBox<T,N>>> tree;

    /**
     * The split function for the construction
     */
    const SplitFunction splitFun;

    /**
     * The depth of the tree
     */
    std::size_t treeDepth = 0;
};

/********** BoundingBox Functions Implementation **********/
template <typename T, std::size_t N>
bool BoundingBox<T,N>::hyperSphereIntersection(const Point &p, const T &radius) const
{
  Point rCenter = T(0.5) * (min + max);
  Point cDist = (p - rCenter).cwiseAbs();
  Point rLen = max - min;
  T r = std::sqrt(radius);

  for(std::size_t dim = 0; dim < N; ++dim)
  {
    if(cDist(dim) > T(0.5) * rLen(dim) + r)
      return false;
    if(cDist(dim) <= rLen(dim))
      return true;
  }

  return (cDist - T(0.5) * rLen).squaredNorm() <= radius;
}

/********** KDTree Functions Implementation *********/
template <typename T, std::size_t N>
KDTree<T,N>::KDTree(KDPointArray &arr, const SplitFunction &splitFun)
  : splitFun(splitFun)
{
  assert(arr.size() >= 2);

  Point bMin = Point::Constant(N, std::numeric_limits<T>::max()), 
        bMax = Point::Constant(N, std::numeric_limits<T>::min());

  for(auto const& n : arr)
  {
    bMin = bMin.cwiseMin(*n);
    bMax = bMax.cwiseMax(*n);
  }

  BoundingBox<T,N> b(bMin, bMax);

  std::size_t treeSize 
    = static_cast<std::size_t>(std::pow(2, 
          static_cast<std::size_t>(std::log(arr.size()) / std::log(2.0)) + 1));

  tree.resize(treeSize, std::make_pair(Point::Zero(), BoundingBox<T,N>()));

  KDTree::makeTree(arr.begin(), arr.end(), b);
}

template <typename T, std::size_t N>
void KDTree<T,N>::makeTree(const ArrayIter &begin,
      const ArrayIter &end,
      const BoundingBox<T,N> box,
      const std::size_t treePos,
      const std::size_t depth)
{
  using BoundingBox = BoundingBox<T, N>;

  if(treePos > tree.size() - 1) return;

  treeDepth = std::max(depth, treeDepth);

  // We call the splitting function to get an iterator
  // to the point on the splitting plane
  const ArrayIter middle = splitFun(begin, end, depth);

  tree[treePos]  = std::make_pair(**middle, box);

  // Left recursion
  BoundingBox lBox = box; lBox.max(depth % N) = (*middle)->operator()(depth % N);
  if(std::distance(begin, middle) > 0)
    makeTree(begin, middle, lBox, 2 * treePos + 1, depth + 1);

  // Right recursion
  BoundingBox rBox = box; rBox.min(depth % N) = (*middle)->operator()(depth % N);
  if(std::distance(middle + 1, end) > 0)
    makeTree(middle + 1, end, rBox, 2 * treePos + 2, depth + 1);
}

template <typename T, std::size_t N>
typename KDTree<T,N>::Point KDTree<T,N>::nearest(const Point &point) const
{
  Point closest = tree[0].first;
  T minDist = std::numeric_limits<T>::max();
  nearest(0, point, closest, minDist);
  return closest;
}

template <typename T, std::size_t N>
void KDTree<T,N>::nearest(const std::size_t node,
    const Point &point,
    Point &closest,
    T &minDist,
    const std::size_t depth) const
{
  if(node > tree.size() - 1) return;
  if(tree[node].first == Point::Zero()) return;

  // Checks if the hypersphere of radius minDist centered on the query point
  // intersects the bounding box. If so, the algorithm continue, otherwise
  // it halts.
  if(!tree[node].second.hyperSphereIntersection(point, minDist))
    return;

  // Check if the current point is closer than the best found so far
  const T dist = (point - tree[node].first).squaredNorm();
  if(dist < minDist)
  {
    minDist = dist;
    closest = tree[node].first;
  }

  // The recursion chooses to explore the left of the tree first
  // if the coordinate dim of the query point lies left of the
  // splitting plane
  const size_t dim = depth % N;
  if(point(dim) <= tree[node].first.operator()(dim))
  {
    nearest(2 * node + 1, point, closest, minDist, depth + 1);
    nearest(2 * node + 2, point, closest, minDist, depth + 1);
  }
  else
  {
    nearest(2 * node + 2, point, closest, minDist, depth + 1);
    nearest(2 * node + 1, point, closest, minDist, depth + 1);
  }
}
