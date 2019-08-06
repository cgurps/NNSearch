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

/**
 * @struct KDPoint
 * @brief Euclidean point of dimension N.
 * @tparam T the type of point (must be a real floating point type)
 * @tparam N the dimension of the point (must be greater than zero)
 */
template <class T, std::size_t N>
struct KDPoint
{
  static_assert(N > 0, "The dimension of a KDPoint must be greater than 0.");
  static_assert(!std::numeric_limits<T>::is_integer, "T must be a real floating point type");

  /**
   * Default constructor for the KDPoint
   */
  KDPoint<T,N>() 
  {}

  /**
   * r-value constructor for the KDPoint
   *
   * @param t the r-value reference
   */
  KDPoint<T,N>(std::array<T,N> &&t)
    : point(std::move(t))
  {}

  /**
   * reference constructor for the KDPoint
   *
   * @param t the reference
   */
  KDPoint<T,N>(std::array<T,N> &t)
    : point(std::move(t))
  {}

  /**
   * assignement operator
   */
  KDPoint<T,N>& operator=(KDPoint p)
  {
    std::swap(point, p.point);
    return *this;
  }

  /**
   * Default destructor
   */
  ~KDPoint<T,N>() = default;

  /**
   * The internal storage of the point coordinates
   */
  std::array<T, N> point;
};

/**
 * L2 distance between two euclidean points
 * @param a the first point
 * @param b the second point
 * @return the distance between a and b
 */
template<class T, std::size_t N>
T distance(const std::array<T, N> &a, const std::array<T, N> &b);

/**
 * @struct BoundingBox
 * @brief A simple bounding box
 * @tparam T the type of point (must be a real floating point type)
 * @tparam N the dimension of the point (must be greater than zero)
 */
template <class T, std::size_t N>
struct BoundingBox
{
  BoundingBox()
  {}

  /**
   * Constructor of the bounding box
   * @param min the lower end of the bounding box
   * @param max the upper end of the bounding box
   */
  BoundingBox(const std::array<T,N> &min, const std::array<T,N> &max)
    : min(std::move(min)), max(std::move(max))
  {}

  /**
   * rvalue constructor of the bounding box
   * @param min the lower end of the bounding box
   * @param max the upper end of the bounding box
   */
  BoundingBox(const std::array<T,N> &&min, const std::array<T,N> &&max)
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
  bool hyperSphereIntersection(const std::array<T,N> &p, const T &radius) const;

  /**
   * The lower end of the bounding box
   */
  std::array<T,N> min;

  /**
   * The upper end of the bounding box
   */
  std::array<T,N> max;
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
   *  Unique pointer alias of KDNode
   */
  using KDNodeUPtr = std::unique_ptr<const KDNode<T,N>>;

  /**
   *  Shared pointer alias of KDPoint
   */
  using KDPointSPtr = std::shared_ptr<const KDPoint<T,N>>;

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
     * Alias for the KDNode unique pointer
     */
    using KDNodeUPtr = std::unique_ptr<const KDNode<T,N>>;

    /**
     * Alias for the KDPoint shared pointer
     */
    using KDPointSPtr = std::shared_ptr<const KDPoint<T,N>>;

    /**
     * Alias for the input KDPoint shared pointer vector
     */
    using KDPointArray = typename std::vector<KDPointSPtr>;

    /**
     * Alias for the KDPointArray iterator
     */
    using ArrayIter = typename KDPointArray::iterator;

    /**
     * Alias for the split function used for the construction of the tree
     */
    using SplitFunction = std::function<ArrayIter(const ArrayIter&, 
        const ArrayIter&, 
        const std::size_t)>;

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
    const KDPoint<T, N> nearest(const std::array<T, N> &point) const;

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
        const std::array<T, N> &point,
        KDPoint<T, N> &closest,
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

/********** KDPoint Functions Implementation *********/
template<class T, std::size_t N>
T distance(const std::array<T, N> &a, const std::array<T, N> &b)
{
  T d(0.0);
  for(std::size_t dim = 0; dim < N; ++dim)
    d += (a[dim] - b[dim]) * (a[dim] - b[dim]);

  assert(d >= T(0));

  return std::sqrt(d);
}

/********** BoundingBox Functions Implementation **********/
template <typename T, std::size_t N>
bool BoundingBox<T,N>::hyperSphereIntersection(const std::array<T,N> &p, const T &radius) const
{
  T cornerDistance(0);
  for(std::size_t dim = 0; dim < N; ++dim)
  {
    T rDimCenter = T(0.5) * (min[dim] + max[dim]);
    T cDimDist = std::abs(p[dim] - rDimCenter);
    T rLen = max[dim] - min[dim];
    if(cDimDist > T(0.5) * rLen + radius)
      return false;
    if(cDimDist <= rLen)
      return true;

    cornerDistance += std::pow(cDimDist - T(0.5) * rLen, 2);
  }
  
  return cornerDistance <= radius * radius;
}

/********** KDTree Functions Implementation *********/
template <typename T, std::size_t N>
KDTree<T,N>::KDTree(KDPointArray &arr, const SplitFunction &splitFun)
  : splitFun(splitFun)
{
  assert(arr.size() >= 2);

  std::array<T,N> bMin, bMax;
  std::fill(bMin.begin(), bMin.end(), std::numeric_limits<T>::max());
  std::fill(bMax.begin(), bMax.end(), std::numeric_limits<T>::min());

  KDPointArray internalArr;
  for(auto const& n : arr)
  {
    for(std::size_t dim = 0; dim < N; ++dim)
    {
      bMin[dim] = std::min(n->point[dim], bMin[dim]);
      bMax[dim] = std::max(n->point[dim], bMax[dim]);
    }
    internalArr.push_back(n);
  }

  root = KDTree::makeTree(internalArr.begin(), 
      internalArr.end(), 
      BoundingBox(bMin, bMax));
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
  BoundingBox lBox = box; lBox.max[depth % N] = (*middle)->point[depth % N];
  if(std::distance(begin, middle) > 0)
    left = makeTree(begin, middle, lBox, depth + 1);
  else left = nullptr;

  KDNodeUPtr right;
  BoundingBox rBox = box; rBox.min[depth % N] = (*middle)->point[depth % N];
  if(std::distance(middle + 1, end) > 0)
    right = makeTree(middle + 1, end, rBox, depth + 1);
  else right = nullptr;

  return std::make_unique<const KDNode<T,N>>(*middle, left, right, box);
}

template <typename T, std::size_t N>
const KDPoint<T,N> KDTree<T,N>::nearest(const std::array<T, N> &point) const
{
  KDPoint<T, N> closest({0.0, 0.0});
  T minDist = std::numeric_limits<T>::max();
  nearest(root, point, closest, minDist);
  return closest;
}

template <typename T, std::size_t N>
void KDTree<T,N>::nearest(const KDNodeUPtr &node,
    const std::array<T, N> &point,
    KDPoint<T, N> &closest,
    T &minDist,
    const std::size_t depth) const
{
  if(node == nullptr || node->isLeaf())
    return;

  if(!node->bb.hyperSphereIntersection(point, minDist))
    return;

  assert(node->value != nullptr);

  const T dist = distance(point, node->value->point); 
  if(dist < minDist)
  {
    minDist = dist;
    closest = *(node->value);
  }

  const size_t dim = depth % N;
  if(point[dim] <= node->value->point[dim])
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
