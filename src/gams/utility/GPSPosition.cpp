/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names Carnegie Mellon University, "SEI and/or Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/

#include <cmath>
#include <sstream>
#include <iomanip>

#include "gams/utility/GPSPosition.h"
#include "gams/utility/Position.h"

#include "gams/loggers/GlobalLogger.h"

using std::stringstream;

gams::utility::GPSPosition::GPSPosition (double init_lat, double init_lon, double init_alt)
: Position (init_lat, init_lon, init_alt)
{
}

gams::utility::GPSPosition::GPSPosition (const Position & rhs)
: Position (rhs.x, rhs.y, rhs.z)
{
}

gams::utility::GPSPosition::GPSPosition (const GPSPosition & rhs)
: Position (rhs.x, rhs.y, rhs.z)
{
}

gams::utility::GPSPosition::~GPSPosition ()
{
}


void
gams::utility::GPSPosition::operator= (const GPSPosition & rhs)
{
  if (this != &rhs)
  {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
  }
}

bool
gams::utility::GPSPosition::operator< (const GPSPosition& rhs) const
{
  if (this->x < rhs.x)
    return true;
  else if (this->x == rhs.x)
  {
    if (this->y < rhs.y)
      return true;
    else if (this->y == rhs.y)
      return this->z < rhs.z;
  }
  return true;
}

bool
gams::utility::GPSPosition::operator== (const GPSPosition & rhs) const
{
  return this->x == rhs.x && this->y == rhs.y && this->z == rhs.z;
}

bool
gams::utility::GPSPosition::operator== (
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return rhs.size () == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::GPSPosition::operator== (
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return rhs.size () == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::GPSPosition::operator!= (const GPSPosition & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::GPSPosition::operator!= (
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::GPSPosition::operator!= (
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::GPSPosition::approximately_equal (const GPSPosition & rhs,
  const double & epsilon) const
{
  return this->distance_to(rhs) <= epsilon;
}

void
gams::utility::GPSPosition::direction_to (const GPSPosition& rhs, 
  double& phi) const
{
  /**
   * Use rhumb line
   * phi = lat, lambda = long
   * Taken from: http://www.movable-type.co.uk/scripts/latlong.html
   */
  double phi_1 = DEG_TO_RAD (latitude ());
  double phi_2 = DEG_TO_RAD (rhs.latitude ());
  double del_lambda = DEG_TO_RAD (rhs.longitude () - longitude ());
  if (fabs (del_lambda) > M_PI)
    del_lambda = (del_lambda > 0) ? (-2 * M_PI + del_lambda) : (2 * M_PI + del_lambda);
  double del_psi = log (tan (phi_2 / 2 + M_PI / 4) / tan(phi_1 / 2 + M_PI / 4));
  double theta = atan2 (del_lambda, del_psi);
  theta = fmod (theta + 2 * M_PI, 2 * M_PI);

  phi = theta;
}

double
gams::utility::GPSPosition::distance_to (const GPSPosition & rhs) const
{
  /**
   * We make the assumption that the curvature of the Earth is insignificant
   * over the distances we will be covering.
   */
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // calculate north/south distance
  const double ns_dif = EARTH_CIRCUMFERENCE * (this->x - rhs.x) / 360.0;

  // calculate east/west distance
  const double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (this->x));
  const double circumference = 2 * r_prime * M_PI;
  const double ew_dif = circumference * (this->y - rhs.y) / 360.0;

  // calculate altitude difference
  const double alt_dif = this->z - rhs.z;

  // use distance formula
  const double dist =
    sqrt (pow (ns_dif, 2.0) + pow (ew_dif, 2.0) + pow (alt_dif, 2.0));

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::utility::GPSPosition::distance_to:" \
    " distance from \"%s\" to \"%s\" is %f\n", to_string ().c_str (),
    rhs.to_string ().c_str (), dist);

  return dist;
}

std::string
gams::utility::GPSPosition::to_string (const std::string & delimiter,
  const unsigned int precision) const
{
  stringstream buffer;
  buffer << std::setprecision(precision);
  buffer << latitude ();
  buffer << delimiter;
  buffer << longitude ();
  buffer << delimiter;
  buffer << altitude ();

  return buffer.str ();
}

gams::utility::GPSPosition
gams::utility::GPSPosition::to_gps_position (
  const Position & source, const GPSPosition & ref)
{
  GPSPosition ret;

  // assume the Earth is a perfect sphere
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // convert the latitude/x coordinates
  ret.x = source.x * 360.0 / EARTH_CIRCUMFERENCE + ref.x;
  
  // assume the meters/degree longitude is constant throughout environment
  // convert the longitude/y coordinates
  double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (ref.x));
  double circumference = 2 * r_prime * M_PI;
  ret.y = source.y / circumference * 360 + ref.y;

  // keep same altitude
  ret.z = ref.z + source.z;

  return ret;
}

gams::utility::Position
gams::utility::GPSPosition::to_position (const GPSPosition& ref) const
{
  /**
   * Make the assumption that the Earth is a perfect sphere and that the 
   * curvature of the Earth over the two points is insignificant
   */
  Position ret;

  // assume the Earth is a perfect sphere
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // convert the latitude/x coordinates
  // VREP uses y for latitude
  ret.x = (this->x - ref.x) / 360.0 * EARTH_CIRCUMFERENCE;
  
  // assume the meters/degree longitude is constant throughout environment
  // convert the longitude/y coordinates
  // VREP uses x for longitude
  double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (this->x));
  double circumference = 2 * r_prime * M_PI;
  ret.y = (this->y - ref.y) / 360.0 * circumference;

  // keep same altitude
  ret.z = this->z - ref.z;

  return ret;
}

gams::utility::GPSPosition
gams::utility::GPSPosition::from_string (const std::string & s)
{
  GPSPosition temp;
  sscanf (s.c_str (), "%lf%*s%lf%*s%lf", &temp.x, &temp.y, &temp.z);
  return temp;
}

void
gams::utility::GPSPosition::to_container (
  madara::knowledge::containers::DoubleArray & target) const
{
  target.set (0, latitude ());
  target.set (1, longitude ());
  target.set (2, altitude ());
}

void
gams::utility::GPSPosition::from_container (
  madara::knowledge::containers::DoubleArray & source)
{
  if (source.size () >= 3)
  {
    latitude (source[0]);
    longitude (source[1]);
    altitude (source[2]);
  }
}

void
gams::utility::GPSPosition::to_container (
  madara::knowledge::containers::NativeDoubleArray & target) const
{
  target.set (0, latitude ());
  target.set (1, longitude ());
  target.set (2, altitude ());
}

void
gams::utility::GPSPosition::from_container (
  madara::knowledge::containers::NativeDoubleArray & source)
{
  if (source.size () >= 3)
  {
    latitude (source[0]);
    longitude (source[1]);
    altitude (source[2]);
  }
}
