/**
 * @file query.cpp
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 * @brief Main for the executable query
 */

/**
 * @dir src
 * @brief The source directory
 */

#include "KDTree.h"
#include "SplitFunctions.h"
#include "OBJ_Loader.h"
#include "ProgramOptions.h"

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Point;
typedef KDTree<Scalar, 3>  Tree;

/**
* The main function takes as input a 3D point and a filename for the OBJ.
* It performs then the query of the 3D point against the loaded OBJ and ouputs
* the result in the standart output.
*/
int main(int argc, char** argv)
{
  ProgramOptions options = parseOptions(argc, argv);

  objl::Loader loader;
  bool loadout = loader.LoadFile(options.filename);
  
  if(loadout)
  {
    objl::Mesh mesh = loader.LoadedMeshes[0];

    std::vector<Point*> points;
    for(std::size_t i = 0; i < mesh.Vertices.size(); ++i)
    {
      Point *p = new Point({mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z});
      points.push_back(p);
    }

    Tree tree(points, median<Scalar,3>);
    auto p = tree.nearest(options.point);

    std::cout << p(0) << " " << p(1) << " " << p(2) << std::endl;
  }

  return 0;
}
