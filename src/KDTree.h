#pragma once

#include <utility>
#include <array>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <type_traits>

template <class T, std::size_t N>
class KDPoint
{
  public:
    KDPoint<T,N>() 
    {
      static_assert(N > 0, "The dimension of a KDPoint must be greater than 0.");
      static_assert(!std::numeric_limits<T>::is_integer, "T must be a real floating point type");
    }

    KDPoint<T,N>(std::array<T,N> &&t)
      : point(std::move(t))
    {
      static_assert(N > 0, "The dimension of a KDPoint must be greater than 0.");
      static_assert(!std::numeric_limits<T>::is_integer, "T must be a real floating point type");
    }

    KDPoint<T,N>(std::array<T,N> &t)
      : point(std::move(t))
    {
      static_assert(N > 0, "The dimension of a KDPoint must be greater than 0.");
      static_assert(!std::numeric_limits<T>::is_integer, "T must be a real floating point type");
    }

    ~KDPoint<T,N>() = default;

    T operator[](const std::size_t &dim)
    {
      assert(dim >= 0);
      return point[dim];
    }

    std::array<T, N> point;
};

template<class T, std::size_t N>
T distance(const std::array<T, N> &a, const std::array<T, N> &b);

template <typename T, std::size_t N>
std::ostream &operator<<(std::ostream &out, const KDPoint<T, N>& p);

template <class T, std::size_t N>
class KDNode
{
  using KDNodeUPtr = std::unique_ptr<const KDNode<T,N>>;
  using KDPointSPtr = std::shared_ptr<const KDPoint<T,N>>;

  public:
    KDNode(KDPointSPtr p, 
           KDNodeUPtr &lhs, 
           KDNodeUPtr &rhs, 
           const std::size_t &depth)
      : value(p),
        left(std::move(lhs)), 
        right(std::move(rhs)), 
        depth(depth)
    {
      assert(depth >= 0);
    }

    KDNode(KDPointSPtr p, 
           const std::size_t &depth)
      : value(p),
        depth(depth)
    {
      assert(depth >= 0);
    }

    KDNode() = delete;
    KDNode(const KDNode &) = delete;
    KDNode &operator = (const KDNode &) = delete;

    ~KDNode() = default;

    bool isLeaf() const
    {
      return value == nullptr;
    }

    const KDNodeUPtr left, right;
    const KDPointSPtr value;
    const std::size_t depth;
};

template <typename T, std::size_t N>
std::ostream &operator<<(std::ostream &out, const KDNode<T, N>& n);

template <class T, std::size_t N>
class KDTree
{
  using KDNodeUPtr = std::unique_ptr<const KDNode<T,N>>;
  using KDPointSPtr = std::shared_ptr<const KDPoint<T,N>>;
  using KDPointArray = typename std::vector<KDPointSPtr>;

  public:
    
    KDTree() = delete;
    KDTree(const KDTree &) = delete;
    KDTree &operator = (const KDTree &) = delete;
    ~KDTree() = default;

    KDTree(KDPointArray &arr);

    void display() 
    { 
      assert(root != nullptr);
      displayTree(root); 
    }

    const KDPoint<T, N> nearest(const std::array<T, N> &point) const;

  private:
    KDNodeUPtr makeTree(const typename KDPointArray::iterator &begin, 
        const typename KDPointArray::iterator &end,
        const std::size_t level = 0); 

    void nearest(const KDNodeUPtr &node,
        const std::array<T, N> &point,
        KDPoint<T, N> &closest,
        T &minDist) const;

    void displayTree(const KDNode<T,N> *ptr); 

    KDNodeUPtr root;
};

#include "KDTree.ih"
