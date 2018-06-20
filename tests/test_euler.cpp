#include <iostream>
#include <math.h>
#include "gams/pose/ReferenceFrame.h"
#include "gams/pose/Euler.h"
#include "gams/pose/Orientation.h"

using namespace gams::pose;

using namespace gams::pose::euler;
using std::cout;
using std::endl;

int main(int, char *[])
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
    EulerZYX e(15, 45, 60, degrees);
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
    EulerExtrXYZ e(60, 45, 15, degrees);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;

    EulerExtrXYZ e2(q);
    cout << e2 << endl;

    Quaternion q2(e2.to_quat());
    cout << q2 << endl;
  }
  {
    EulerZYX e(1.0/24, 1.0/8, 1.0/6, revolutions);
    Quaternion q(e.to_quat());

    cout << e << endl;
    cout << q << endl;
  }

  {
    EulerExtrXYZ e(60, 45, 15, degrees);
    Orientation r(e.to_orientation());

    cout << r << endl;
  }
  {
    EulerExtrXYZ e(0, 0, 1, radians);
    Orientation r(e.to_orientation());

    cout << r << endl;
  }

  //Euler<conv::Z, conv::X, conv::Y> should_fail(1, 2, 3);
  //cout << should_fail.to_quat() << endl;
  return 0;
}
