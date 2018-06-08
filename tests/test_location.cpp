#include <math.h>
#include <gams/utility/Location.h>
#include "gtest/gtest.h"

using namespace gams::utility;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.0001;

TEST(TestLocation, TestLocation)
{
  Location dloc0(0,0,0);
  Location dloc1(3,4,0);

  EXPECT_NEAR(dloc0.distance_to(dloc1), 5, TEST_epsilon);
  EXPECT_NEAR(dloc1.distance_to(dloc0), 5, TEST_epsilon);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
