#include "functions.h"

int main(int argc, char** argv)
{
  int nbPoints = 10000, nbTestPoints = 4000;

  std::random_device dev;
  std::mt19937 rng(dev());

  test<double, 3>(nbPoints, nbTestPoints, rng);
  test<float, 3>(nbPoints, nbTestPoints, rng);

  return 0;
}
