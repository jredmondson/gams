

#include <iostream>
#include <math.h>

#ifdef GAMS_UTM

#include "gams/utility/GPSFrame.h"
#include "gams/utility/UTMFrame.h"
#include "gams/utility/CartesianFrame.h"

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

using std::cout;
using std::endl;

#endif // GAMS_UTM

int main(int , char **)
{
#ifdef GAMS_UTM

  GPSFrame gps;
  UTMFrame utm(gps);

  Location gloc(gps, -79, 42);
  Location uloc(utm, gloc);

  cout.precision(17);
  cout << gloc << endl;
  cout << uloc << endl;
  cout << uloc.northing() << " " << uloc.hemi() << " " << uloc.easting () << " " << uloc.zone() << uloc.nato_band() << endl;

  Location gloc2(gps, uloc);
  cout << gloc2 << endl;

  CartesianFrame cart(Pose(uloc, Rotation(0, 0, 180, degrees)));
  cout << cart.origin() << endl;
  Location cloc(cart, 10, 10);
  cout << cloc << endl;

  Location ucloc(utm, cloc);
  cout << ucloc << endl;
  cout << ucloc.northing() << " " << ucloc.hemi() << " " << ucloc.easting () << " " << ucloc.zone() << uloc.nato_band() << endl;

  Location uloc2(utm.mk_loc(50000, 17, 1000000, 0));
  cout << uloc2 << endl;
  Location gloc3(gps, uloc2);
  cout << gloc3 << endl;

  Location polar(gps, -80, 86);
  Location upolar(utm, polar);

  cout << polar << endl;
  cout << upolar << endl;

  {
    Pose p(gps, 0, 86);
    LOG(p);
    Pose up(utm, p);
    LOG(up);
    p.lng(45);
    LOG(p);
    Pose up2(utm, p);
    LOG(up2);
    Pose p2(gps, up2);
    LOG(p2);
    p.lng(90);
    Pose up3(utm, p);
    LOG(up3);
  }
  {
    Pose p(gps, 0, -86);
    LOG(p);
    Pose up(utm, p);
    LOG(up);
    p.lng(45);
    LOG(p);
    Pose up2(utm, p);
    LOG(up2);
    Pose p2(gps, up2);
    LOG(p2);
    p.lng(90);
    Pose up3(utm, p);
    LOG(up3);
  }
#endif // GAMS_UTM

  return 0;
}
