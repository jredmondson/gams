#include <iostream>
#include <math.h>
#include "gams/pose/Position.h"

using namespace gams::pose;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.0001;
int gams_fails = 0;

double round_nearest(double in)
{
  return floor(in + 0.5);
}

#define LOG(expr) \
  std::cout << #expr << " == " << (expr) << std::endl

#define TEST(expr, expect) \
  do {\
    double bv = (expr); \
    double v = round_nearest((bv) * 1024)/1024; \
    double e = round_nearest((expect) * 1024)/1024; \
    bool ok = \
      e >= 0 ? (v >= e * (1 - TEST_epsilon) && v <= e * (1 + TEST_epsilon)) \
             : (v >= e * (1 + TEST_epsilon) && v <= e * (1 - TEST_epsilon)); \
    if(ok) \
    { \
      std::cout << #expr << " ?= " << e << "  SUCCESS! got " << bv << std::endl; \
    } \
    else \
    { \
      std::cout << #expr << " ?= " << e << "  FAIL! got " << bv << " instead" << std::endl; \
      gams_fails++; \
    } \
  } while(0)

int main(int , char **)
{
  std::cout.precision(4);
  std::cout << std::fixed;
  std::cout << "Testing Position:" << std::endl;
  Position dloc0(0,0,0);
  Position dloc1(3,4,0);

  LOG(dloc0);
  LOG(dloc1);

  TEST(dloc0.distance_to(dloc1), 5);
  TEST(dloc1.distance_to(dloc0), 5);

  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}
