Main Page {#mainpage}
=========

# Introduction
This piece of code implements a nearest neighbords search algorithm using a k-d tree.
The algorithm works in arbitrary dimension, but suffers from the curse of dimensionality.

## Getting Started
You will first need to clone the repository
```
git clone https://github.com/cgurps/2DFluidSimulation.git [PROJECT_FOLDER]
```
and then init the submodules
```
cd [PROJECT_FOLDER]
git submodule update --init
```
To compile the project, you will need [Boost](https://www.boost.org/) installed on your machine. To project uses CMake to generate the Makefile needed for the compilation. You can use these commands to build the executable

```
mkdir build
cd build
cmake ..
make -j [YOUR_NUMBER_OF_CORES]
```

You can query the program options using `-h`.

## Algorithm
The k-d tree structure implemented here is a balanced binary tree. Each node of the tree contains two smart pointers toward its children and a point lying on the splitting plane used for constructing the tree. The splitting dimension depends on the depth of the tree: we start with dimension zero at depth zero, and then dimension one at depth one and so on. I also store a bounding box of the subtree for each node of the k-d tree.

The nearest neighbors algorithm performs a simple recursive search of the tree. The search space is reduced using two methods:
1. at each recursion, we check if the distance from Q to the current node bounding box is less than the current best distance. If so, we stop the recursion as there is no point in the subtree that can beat the current best point
2. the algorithm chooses to search the left or the right children by checking on which side of the splitting plane Q is. This ensures that we search first the best part of the tree for the nearest neighbors. This also helps reducing the size of the bounding box, which in turns feeds the first method for pruning the search space.

## Implementation
The k-d tree implementation is header only and is contained in `KDtree.h`. The first structure `KDNode` represents one node of the tree and contains two smart pointers to its children and another smart pointer to the node value (ie the point lying on the splitting plane). It also contains its `BoundingBox` used for the nearest neighbor algorithm. The class `KDTree` implements a k-d tree of dimension N and contains a constructor taking a array containing the initial points, and the method for the nearest neighbors query (`nearest(Point)`).

The actual representation of an Euclidean uses the type `Eigen::Matrix<T,N,1>` (a matrix of type `T` with `N` columns and one row) of [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This allows the use of various functions of the library (for example efficient distance computation between points).

The current bottleneck of the implementation is the computation of the distance between the query point Q and the bounding box of a node during the recursion.
The function `hyperSphereIntersection` from the `BoundingBox` is a generalization of the method proposed by e.James from 
[StackOverflow](https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection). 
The idea is to first reduce the computation to one corner of the bounding box, and then checks for the following three cases:
1. the circle is far enough from the bounding box (easy)
2. the circle is close enough from the bounding box (easy)
3. the circle intersects the corner of the bounding box (difficult)

## Timings
I tested timing on Archlinux (linux kernel 5.2.4) using gcc 9.1.0 with optimizations. My CPU is and Intel Core i7-6700K.
The test runs 10000 random queries against the loaded mesh.

| Shape | Nb. of Points | Tree Construction Time | Tree Memory | Average Time |
|:-|:-:|:-:|:-:|:-:|
| Stanford Bunny | 14904 (0.341125mb) | 3.3495 ms| 1.24992mb |  9.92943 µs |
| Stanford Dragon | 2613918 (59.8278mb) | 744.452 ms | 320mb | 680.913 µs |
| Happy Buddha | 3262422 (74.6707mb) | 954.633 ms | 320mb | 549.162 µs |
| Random Points | 15000000 (343.323mb) | 10707.3 ms | 1280mb | 8883.77 µs |
