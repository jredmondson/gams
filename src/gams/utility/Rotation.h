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

#ifndef _GAMS_UTILITY_ROTATION_H_
#define _GAMS_UTILITY_ROTATION_H_

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

#include <gams/utility/Coordinate.h>

#define DEG_TO_RAD(x) (((x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) (((x) * 180) / M_PI)

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    class Quaternion;

    /**
     * Container for Rotation information, not bound to a frame.
     **/
    class Rotation_Vector
    {
    public:
      static const int X_axis = 0;
      static const int Y_axis = 1;
      static const int Z_axis = 2;

      constexpr Rotation_Vector(double rx, double ry, double rz);

      constexpr Rotation_Vector(double x, double y, double z, double angle);

      constexpr Rotation_Vector(int axis_index, double angle);

      constexpr Rotation_Vector();

      constexpr Rotation_Vector(const Rotation_Vector &orig);

      explicit Rotation_Vector(const Quaternion &quat);

      constexpr bool is_invalid() const;

      constexpr bool is_zero() const;

      constexpr bool operator==(const Rotation_Vector &other) const;

      static std::string name();

      constexpr double rx() const;
      constexpr double ry() const;
      constexpr double rz() const;

      double rx(double new_rx);
      double ry(double new_ry);
      double rz(double new_rz);

      typedef Rotation_Vector Base_Type;

      Base_Type &as_vec();

      constexpr const Base_Type &as_vec() const;

      constexpr int size() const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order rx, ry, rz
       **/
      constexpr double get(int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order rx, ry, rz
       **/
      double set(int i, double val);

      friend class Quaternion;
    private:
      double rx_, ry_, rz_;
    };

    /**
     * Represents a rotation or orientation within a reference frame.
     *
     * All rotations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the rotation axis, rotations curve in the direction your
     * fingers are pointing.
     **/
    class Rotation : public Rotation_Vector, public Coordinate<Rotation>
    {
    public:
      /**
       * The direction of rx, ry, and rz is the axis of rotation, with respect
       * to the owning frame's axes.
       * The magnitude of rx, ry, and rz (as a vector) is the amount of rotation
       * in radians.
       **/
      Rotation(double rx, double ry, double rz);

      constexpr Rotation(const Reference_Frame &frame,
                         double rx, double ry, double rz);

      /**
       * (x, y, z) must be a unit vector in the direction of rotation axis.
       * angle is the angle of rotation, in degrees, about that axis
       **/
      Rotation(double x, double y, double z, double angle);

      constexpr Rotation(const Reference_Frame &frame,
               double x, double y, double z, double angle);

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       **/
      Rotation(int axis_index, double angle);

      constexpr Rotation(const Reference_Frame &frame,
                         int axis_index, double angle);

      Rotation();

      constexpr Rotation(const Rotation &orig);

      explicit Rotation(const Quaternion &quat);

      explicit Rotation(const Reference_Frame &frame, const Quaternion &quat);

      Rotation(const Reference_Frame &new_frame, const Rotation &orig);

      double angle_to(const Rotation &target) const;

      using Coordinate<Rotation>::operator==;
    };
  }
}

#include "Rotation.inl"

// Include if not already included
#include <gams/utility/Pose.h>

#endif
