#include "functions.h"

int main(int argc, char** argv)
{
  int nbPoints = 5000, nbTestPoints = 3000;

  std::random_device dev;
  std::mt19937 rng(dev());

  test<double, 20>(nbPoints, nbTestPoints, rng);
  test<float, 20>(nbPoints, nbTestPoints, rng);

  return 0;
}
