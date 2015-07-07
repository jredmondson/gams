/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
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

/**
 * @file Coordinates.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location, Rotation, and Pose classes
 **/

#ifndef _GAMS_UTILITY_QUATERNION_H_
#define _GAMS_UTILITY_QUATERNION_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>
#include <cstdio>

#define DEG_TO_RAD(x) (((x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) (((x) * 180) / M_PI)

namespace gams
{
  namespace utility
  {
    class Rotation_Vector;
    class Location_Vector;

    /**
     * Used internally to implement angle operations.
     * Not reference-frame aware.
     **/
    class Quaternion
    {
    public:
      constexpr Quaternion(double x, double y, double z, double w);

      Quaternion(double rx, double ry, double rz);

      explicit Quaternion(const Rotation_Vector &rot);

      explicit Quaternion(const Location_Vector &loc);

      void from_location_vector(double x, double y, double z);

      void from_location_vector(const Location_Vector &loc);

      void to_location_vector(double &x, double &y, double &z) const;

      void to_location_vector(Location_Vector &loc) const;

      void from_rotation_vector(double rx, double ry, double rz);

      void from_rotation_vector(const Rotation_Vector &rot);

      void to_rotation_vector(double &rx, double &ry, double &rz) const;

      void to_rotation_vector(Rotation_Vector &rot) const;

      static void hamilton_product(
              Quaternion &into, const Quaternion &lhs, const Quaternion &rhs);

      Quaternion &operator*=(const Quaternion &rhs);

      Quaternion operator*(const Quaternion &o) const;

      Quaternion &conjugate();

      constexpr Quaternion operator-() const;

      double inner_product(const Quaternion &o) const;

      double angle_to(const Quaternion &o) const;

      constexpr double x() const;
      constexpr double y() const;
      constexpr double z() const;
      constexpr double w() const;

      double x(double new_x);
      double y(double new_y);
      double z(double new_z);
      double w(double new_w);

    private:
      double x_, y_, z_, w_;
    };
  }
}

#include "Quaternion.inl"

// Include if not already included
#include <gams/utility/Pose.h>

#endif
