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

#ifndef _GAMS_UTILITY_POSE_H_
#define _GAMS_UTILITY_POSE_H_

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

#include <gams/utility/Location.h>
#include <gams/utility/Rotation.h>

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    /**
     * Container for Pose information, not bound to a frame.
     **/
    class Pose_Vector : public Location_Vector, public Rotation_Vector
    {
    protected:
      constexpr Pose_Vector(double x, double y, double z,
                            double rx, double ry, double rz);

      constexpr Pose_Vector(const Location_Vector &loc);

      constexpr Pose_Vector(const Rotation_Vector &rot);

      constexpr Pose_Vector(const Location_Vector &loc,
                            const Rotation_Vector &rot);

      constexpr Pose_Vector();

    public:
      constexpr bool is_invalid() const;

      constexpr bool is_location_zero() const;

      constexpr bool is_rotation_zero() const;

      constexpr bool is_zero() const;

      constexpr bool operator==(const Pose_Vector &other) const;

      static std::string name();

      constexpr int size() const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      constexpr double get(int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double set(int i, double val);

      typedef Pose_Vector Base_Type;

      Base_Type &as_vec();

      constexpr const Base_Type &as_vec() const;

      Location_Vector &as_location_vec();

      constexpr const Location_Vector &as_location_vec() const;

      Rotation_Vector &as_rotation_vec();

      constexpr const Rotation_Vector &as_rotation_vec() const;
    };

    /**
     * Represents a combination of Location and Rotation within a single
     * reference frame.
     **/
    class Pose : public Pose_Vector, public Coordinate<Pose>
    {
    public:
      Pose(double x, double y, double z, double rx, double ry, double rz);

      Pose(double x, double y, double z = 0.0);

      constexpr Pose(const Reference_Frame &frame,
                     double x, double y, double z,
                     double rx, double ry, double rz);

      constexpr Pose(const Reference_Frame &frame,
                     double x, double y, double z = 0.0);

      Pose();

      constexpr Pose(const Location &loc);

      constexpr Pose(const Rotation &rot);

      Pose(const Location_Vector &loc, const Rotation_Vector &rot);

      constexpr Pose(const Reference_Frame &frame,
                     const Location_Vector &loc,
                     const Rotation_Vector &rot);

      /// Precondition: loc.frame == rot.frame
      constexpr Pose(const Location &loc, const Rotation &rot);

      Pose(const Reference_Frame &new_frame, const Rotation &orig);

      double angle_to(const Pose &target) const;

      double angle_to(const Rotation &target) const;

      constexpr operator Location() const;

      constexpr operator Rotation() const;
    };
  }
}

#include "Pose.inl"

// Include if not already included
#include <gams/utility/Reference_Frame.h>

#endif
