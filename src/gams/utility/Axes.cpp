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
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
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
#include "gams/utility/Axes.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

gams::utility::Axes::Axes (
  double init_x, double init_y, double init_z)
: x (init_x), y (init_y), z (init_z)
{
}

gams::utility::Axes::Axes (const Axes & source)
: x (source.x), y (source.y), z (source.z)
{
}

gams::utility::Axes::~Axes ()
{
}

void
gams::utility::Axes::operator= (const Axes & rhs)
{
  if (this != &rhs)
  {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
  }
}

bool
gams::utility::Axes::operator== (const Axes & rhs) const
{
  return this->x == rhs.x && this->y == rhs.y && this->z == rhs.z;
}

bool
gams::utility::Axes::operator== (
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return rhs.size () == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::Axes::operator== (
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return rhs.size () == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::Axes::operator!= (const Axes & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::Axes::operator!= (
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::Axes::operator!= (
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return !(*this == rhs);
}

std::string
gams::utility::Axes::to_string (const std::string & delimiter) const
{
  std::stringstream buffer;
  buffer << x;
  buffer << delimiter;
  buffer << y;
  buffer << delimiter;
  buffer << z;

  return buffer.str ();
}

gams::utility::Axes
gams::utility::Axes::from_string (const std::string & s)
{
  Axes temp;
  if (sscanf (s.c_str (), "%lf%*[^0-9]%lf%*[^0-9]%lf", &temp.x, &temp.y, &temp.z) != 3)
    temp.x = temp.y = temp.z = DBL_MAX;
  return temp;
}

void
gams::utility::Axes::to_container (
  madara::knowledge::containers::DoubleArray & target) const
{
  target.set (0, x);
  target.set (1, y);
  target.set (2, z);
}

void
gams::utility::Axes::from_container (
  madara::knowledge::containers::DoubleArray & source)
{
  if (source.size () >= 3)
  {
    x = source[0];
    y = source[1];
    z = source[2];
  }
}

void
gams::utility::Axes::to_container (
  madara::knowledge::containers::NativeDoubleArray & target) const
{
  target.set (0, x);
  target.set (1, y);
  target.set (2, z);
}

void
gams::utility::Axes::from_container (
  madara::knowledge::containers::NativeDoubleArray & source)
{
  if (source.size () >= 3)
  {
    x = source[0];
    y = source[1];
    z = source[2];
  }
}

gams::utility::Axes
gams::utility::Axes::operator- (const Axes & rhs) const
{
  return Axes (this->x - rhs.x, this->y - rhs.y, this->z - rhs.z);
}

gams::utility::Axes
gams::utility::Axes::operator+ (const Axes & rhs) const
{
  return Axes (this->x + rhs.x, this->y + rhs.y, this->z + rhs.z);
}

gams::utility::Axes
gams::utility::Axes::operator* (const double& scale) const
{
  return Axes (this->x * scale, this->y * scale, this->z * scale);
}
