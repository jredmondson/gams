#include <iostream>
#include <math.h>
#include <gams/pose/ReferenceFrame.h>
#include <gams/pose/Euler.h>
#include <gams/pose/Orientation.h>
#include "gtest/gtest.h"

using namespace gams::pose;

using namespace gams::pose::euler;
using std::cout;
using std::endl;

TEST(TestEuler, TestXYZtoQuaternion)
{
    EulerXYZ e(0, 0, M_PI/2);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(q.x(), 0);
    EXPECT_FLOAT_EQ(q.y(), 0);
    EXPECT_FLOAT_EQ(q.z(), 0.707107);
    EXPECT_FLOAT_EQ(q.w(), 0.707107);

    EulerXYZ e2(q);
    EXPECT_FLOAT_EQ(e2.a(), 0);
    EXPECT_FLOAT_EQ(e2.b(), 0);
    EXPECT_FLOAT_EQ(e2.c(), M_PI/2);
}

TEST(TestEuler, TestXYZtoQuaternionToXYZ)
{
    EulerXYZ e(M_PI/4, M_PI/8, M_PI/2);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(q.x(), 0.39284748);
    EXPECT_FLOAT_EQ(q.y(), -0.13794969);
    EXPECT_FLOAT_EQ(q.z(), 0.69352);
    EXPECT_FLOAT_EQ(q.w(), 0.587938);

    EulerXYZ e2(q);
    EXPECT_FLOAT_EQ(e2.a(), M_PI/4);
    EXPECT_FLOAT_EQ(e2.b(), M_PI/8);
    EXPECT_FLOAT_EQ(e2.c(), M_PI/2);
}

TEST(TestEuler, TestZYXToQuaternionAndBack)
{
    EulerZYX e(M_PI/4, M_PI/8, M_PI/2);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(q.x(), 0.58793777);
    EXPECT_FLOAT_EQ(q.y(), 0.39284748);
    EXPECT_FLOAT_EQ(q.z(), 0.13794969);
    EXPECT_FLOAT_EQ(q.w(), 0.69351995);

    EulerZYX e2(q);
    EXPECT_FLOAT_EQ(e2.a(), M_PI/4);
    EXPECT_FLOAT_EQ(e2.b(), M_PI/8);
    EXPECT_FLOAT_EQ(e2.c(), M_PI/2);
}

TEST(TestEuler, TestQuaternionZYXDegreesToQuat)
{
    EulerZYX e(15, 45, 60, degrees);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(e.a(), 0.2617994);
    EXPECT_FLOAT_EQ(e.b(), 0.785398);
    EXPECT_FLOAT_EQ(e.c(), 1.0471976);

    EXPECT_FLOAT_EQ(q.x(), 0.41472965);
    EXPECT_FLOAT_EQ(q.y(), 0.38887352);
    EXPECT_FLOAT_EQ(q.z(), -0.085270345);
    EXPECT_FLOAT_EQ(q.w(), 0.818233);

    EulerZYX e2(q);
    EXPECT_FLOAT_EQ(e2.a(), 0.2617994);
    EXPECT_FLOAT_EQ(e2.b(), 0.785398);
    EXPECT_FLOAT_EQ(e2.c(), 1.0471976);

    EulerXYZ e3(e2);
    EXPECT_FLOAT_EQ(e3.a(), 1.1277055);
    EXPECT_FLOAT_EQ(e3.b(), 0.6012215);
    EXPECT_FLOAT_EQ(e3.c(), -0.59481835);

    Quaternion q2(e3.to_quat());
    EXPECT_FLOAT_EQ(q2.x(), 0.41472965);
    EXPECT_FLOAT_EQ(q2.y(), 0.38887352);
    EXPECT_FLOAT_EQ(q2.z(), -0.085270345);
    EXPECT_FLOAT_EQ(q2.w(), 0.818233);
}

TEST(TestEuler, TestExtrZYZ)
{
    EulerExtrXYZ e(60, 45, 15, degrees);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(e.a(), 1.0471976);
    EXPECT_FLOAT_EQ(e.b(), 0.785398);
    EXPECT_FLOAT_EQ(e.c(), 0.2617994);

    EXPECT_FLOAT_EQ(q.x(), 0.41472965);
    EXPECT_FLOAT_EQ(q.y(), 0.38887352);
    EXPECT_FLOAT_EQ(q.z(), -0.085270345);
    EXPECT_FLOAT_EQ(q.w(), 0.818233);

    EulerExtrXYZ e2(q);
    EXPECT_FLOAT_EQ(e2.a(), 1.0471976);
    EXPECT_FLOAT_EQ(e2.b(), 0.785398);
    EXPECT_FLOAT_EQ(e2.c(), 0.2617994);

    Quaternion q2(e2.to_quat());
    EXPECT_FLOAT_EQ(q2.x(), 0.41472965);
    EXPECT_FLOAT_EQ(q2.y(), 0.38887352);
    EXPECT_FLOAT_EQ(q2.z(), -0.085270345);
    EXPECT_FLOAT_EQ(q2.w(), 0.818233);
}

TEST(TestEuler, TestRevolutions)
{
    EulerZYX e(1.0/24, 1.0/8, 1.0/6, revolutions);
    Quaternion q(e.to_quat());

    EXPECT_FLOAT_EQ(e.a(), 0.2617994);
    EXPECT_FLOAT_EQ(e.b(), 0.785398);
    EXPECT_FLOAT_EQ(e.c(), 1.0471976);

    EXPECT_FLOAT_EQ(q.x(), 0.41472965);
    EXPECT_FLOAT_EQ(q.y(), 0.38887352);
    EXPECT_FLOAT_EQ(q.z(), -0.085270345);
    EXPECT_FLOAT_EQ(q.w(), 0.818233);
}

TEST(TestEuler, TestOrientationDegrees)
{
    EulerExtrXYZ e(60, 45, 15, degrees);
    Orientation r(e.to_orientation());
    EXPECT_FLOAT_EQ(r.rx(), 0.883679);
    EXPECT_FLOAT_EQ(r.ry(), 0.82858646);
    EXPECT_FLOAT_EQ(r.rz(), -0.18168852);
}

TEST(TestEuler, TestOrientationRadians)
{
    EulerExtrXYZ e(0, 0, 1, radians);
    Orientation r(e.to_orientation());
    EXPECT_EQ(r.rx(), 0);
    EXPECT_EQ(r.ry(), 0);
    EXPECT_EQ(r.rz(), 1);
}

//Euler<conv::Z, conv::X, conv::Y> should_fail(1, 2, 3);
//cout << should_fail.to_quat() << endl;

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
