#pragma once

/**
 * @file KDTree.h
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
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

/**
 * @struct KDNode
 * @brief represents a KD-Tree node
 *
 * A KDNode contains a shared pointer to a KDPoint representing
 * its splitting point and two unique pointer to its left and right
 * child. A KDNode becomes a leaf when both children are empty.
 *
 * @tparam T the type of point (must be a real floating point type)
 * @tparam N the dimension of the point (must be greater than zero)
 */
template <class T, std::size_t N>
struct KDNode
{
  /**
   * Alias for the internal point structure
   */
  typedef typename Eigen::Matrix<T,N,1> Point;

  /**
   *  Unique pointer alias of KDNode
   */
  typedef std::unique_ptr<const KDNode<T,N>> KDNodeUPtr;

  /**
   *  Shared pointer alias of KDPoint
   */
  typedef std::shared_ptr<const Point> KDPointSPtr;

  /**
   * Internal node constructor
   * @param p the splitting point of the node
   * @param lhs the left child of the node
   * @param rhs the right child of the node
   * @param b the node bounding box
   */
  KDNode(KDPointSPtr p, KDNodeUPtr &lhs, KDNodeUPtr &rhs, BoundingBox<T,N> b) 
    : left(std::move(lhs)), right(std::move(rhs)), value(p), bb(b) 
  {}

  /**
   * Deletion of default constructor
   */
  KDNode() = delete;

  /**
   * Deletion of copy constructor
   */
  KDNode(const KDNode &) = delete;

  /**
   * Deletion of assignement operator
   */
  KDNode &operator= (const KDNode &) = delete;

  /**
   * Default destructor
   */
  ~KDNode() = default;

  /**
   * checks if the node is a leaf
   */
  bool isLeaf() const { return value == nullptr; }

  /**
   * Left child
   */
  const KDNodeUPtr left;

  /**
   * Right child
   */
  const KDNodeUPtr right;

  /**
   * The node point 
   */
  const KDPointSPtr value;

  /**
   * Node bounding box
   */
  const BoundingBox<T,N> bb;
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
     * Alias for the KDNode unique pointer
     */
    typedef std::unique_ptr<const KDNode<T,N>> KDNodeUPtr;

    /**
     * Alias for the KDPoint shared pointer
     */
    typedef std::shared_ptr<const Point> KDPointSPtr;

    /**
     * Alias for the input KDPoint shared pointer vector
     */
    typedef typename std::vector<KDPointSPtr> KDPointArray;

    /**
     * Alias for the KDPointArray iterator
     */
    typedef typename KDPointArray::iterator ArrayIter;

    /**
     * Alias for the split function used for the construction of the tree
     */
    typedef std::function<ArrayIter(const ArrayIter&, 
        const ArrayIter&, 
        const std::size_t)> SplitFunction;

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
      return sizeof(KDNode<T,N>) * (std::pow(2, treeDepth + 1) - 1);
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
    KDNodeUPtr makeTree(const ArrayIter &begin, 
        const ArrayIter &end,
        const BoundingBox<T,N> box,
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
    void nearest(const KDNodeUPtr &node,
        const Point &point,
        Point &closest,
        T &minDist,
        const std::size_t depth = 0) const;

    /**
     * The root of the tree
     */
    KDNodeUPtr root;

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

  for(std::size_t dim = 0; dim < N; ++dim)
  {
    if(cDist(dim) > T(0.5) * rLen(dim) + radius)
      return false;
    if(cDist(dim) <= rLen(dim))
      return true;
  }
  
  return (cDist - T(0.5) * rLen).squaredNorm() <= radius * radius;
}

/********** KDTree Functions Implementation *********/
template <typename T, std::size_t N>
KDTree<T,N>::KDTree(KDPointArray &arr, const SplitFunction &splitFun)
  : splitFun(splitFun)
{
  assert(arr.size() >= 2);

  Point bMin = Point::Constant(N, std::numeric_limits<T>::max()), 
        bMax = Point::Constant(N, std::numeric_limits<T>::min());

  KDPointArray internalArr;
  for(auto const& n : arr)
  {
    bMin = bMin.cwiseMin(*n);
    bMax = bMax.cwiseMax(*n);
    internalArr.push_back(n);
  }

  BoundingBox<T,N> b(bMin, bMax);

  root = KDTree::makeTree(internalArr.begin(), 
      internalArr.end(), 
      b);
}

template <typename T, std::size_t N>
typename KDTree<T,N>::KDNodeUPtr KDTree<T,N>::makeTree(const ArrayIter &begin, 
      const ArrayIter &end,
      const BoundingBox<T,N> box,
      const std::size_t depth)
{
  assert(depth >= 0);

  treeDepth = std::max(depth, treeDepth);

  const ArrayIter middle = splitFun(begin, end, depth);

  KDNodeUPtr left;
  BoundingBox lBox = box; lBox.max(depth % N) = (*middle)->operator()(depth % N);
  if(std::distance(begin, middle) > 0)
    left = makeTree(begin, middle, lBox, depth + 1);
  else left = nullptr;

  KDNodeUPtr right;
  BoundingBox rBox = box; rBox.min(depth % N) = (*middle)->operator()(depth % N);
  if(std::distance(middle + 1, end) > 0)
    right = makeTree(middle + 1, end, rBox, depth + 1);
  else right = nullptr;

  return std::make_unique<const KDNode<T,N>>(*middle, left, right, box);
}

template <typename T, std::size_t N>
typename KDTree<T,N>::Point KDTree<T,N>::nearest(const Point &point) const
{
  Point closest;
  T minDist = std::numeric_limits<T>::max();
  nearest(root, point, closest, minDist);
  return closest;
}

template <typename T, std::size_t N>
void KDTree<T,N>::nearest(const KDNodeUPtr &node,
    const Point &point,
    Point &closest,
    T &minDist,
    const std::size_t depth) const
{
  if(node == nullptr || node->isLeaf())
    return;

  if(!node->bb.hyperSphereIntersection(point, minDist))
    return;

  assert(node->value != nullptr);

  const T dist = (point - *(node->value)).norm();
  if(dist < minDist)
  {
    minDist = dist;
    closest = *(node->value);
  }

  const size_t dim = depth % N;
  if(point(dim) <= node->value->operator()(dim))
  {
    nearest(node->left , point, closest, minDist, depth + 1);
    nearest(node->right, point, closest, minDist, depth + 1);
  }
  else
  {
    nearest(node->right, point, closest, minDist, depth + 1);
    nearest(node->left , point, closest, minDist, depth + 1);
  }
}
