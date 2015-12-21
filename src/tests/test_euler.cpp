#include <iostream>
#include <math.h>
#include <gams/utility/ReferenceFrame.h>
#include <gams/utility/Euler.h>
#include <gams/utility/GPSFrame.h>

using namespace gams::utility;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.0001;

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
    } \
  } while(0)

using namespace gams::utility::euler;
using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
  {
    EulerXYZ e(0, 0, M_PI/2);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerXYZ e2(q);
    cout << e2 << endl;
  }
  {
    EulerXYZ e(M_PI/4, M_PI/8, M_PI/2);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerXYZ e2(q);
    cout << e2 << endl;
  }
  {
    EulerZYX e(M_PI/4, M_PI/8, M_PI/2);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerZYX e2(q);
    cout << e2 << endl;
  }
  {
    EulerZYX e(degrees, 15, 45, 60);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerZYX e2(q);
    cout << e2 << endl;

    EulerXYZ e3(e2);
    cout << e3 << endl;

    Quaternion q2(e3.to_quat());
    cout << q2 << endl;
  }
  {
    EulerExtrXYZ e(degrees, 60, 45, 15);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerExtrXYZ e2(q);
    cout << e2 << endl;

    Quaternion q2(e2.to_quat());
    cout << q2 << endl;
  }
  {
    EulerZYX e(revolutions, 1.0/24, 1.0/8, 1.0/6);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;
  }

  {
    EulerExtrXYZ e(degrees, 60, 45, 15);
    Rotation r(e.to_rotation());

    cout << r << endl;
  }
  {
    EulerExtrXYZ e(radians, 0, 0, 1);
    Rotation r(e.to_rotation());

    cout << r << endl;
  }

  //Euler<conv::Z, conv::X, conv::Y> should_fail(1, 2, 3);
  //cout << should_fail.to_quat() << endl;
  return 0;
}
