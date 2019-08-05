#include "functions.h"

int main(int argc, char** argv)
{
  std::random_device dev;
  std::mt19937 rng(dev());

  testOBJ("../shapes/bunny.obj", 1000, rng);
  testOBJ("../shapes/dragon.obj", 100, rng);
  testOBJ("../shapes/buddha.obj", 100, rng);

  return 0;
}
