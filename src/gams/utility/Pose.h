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
      Pose_Vector(double x, double y, double z, double rx, double ry, double rz)
        : Location_Vector(x, y, z), Rotation_Vector(rx, ry, rz) {}

      Pose_Vector(const Location_Vector &loc)
        : Location_Vector(loc), Rotation_Vector(0, 0, 0) {}

      Pose_Vector(const Rotation_Vector &rot)
        : Location_Vector(0, 0, 0), Rotation_Vector(rot) {}

      Pose_Vector(const Location_Vector &loc, const Rotation_Vector &rot)
        : Location_Vector(loc), Rotation_Vector(rot) {}

      Pose_Vector() : Location_Vector(), Rotation_Vector() {}

    public:
      bool is_invalid() const
      {
        return Location_Vector::is_invalid() || Rotation_Vector::is_invalid();
      }

      bool is_location_zero() const
      {
        return as_location_vec().is_zero();
      }

      bool is_rotation_zero() const
      {
        return as_rotation_vec().is_zero();
      }

      bool is_zero() const
      {
        return is_location_zero() && is_rotation_zero();
      }

      bool operator==(const Pose_Vector &other) const
      {
        return as_location_vec() == other.as_location_vec() &&
               as_rotation_vec() == other.as_rotation_vec();
      }

      static std::string name()
      {
        return "Pose";
      }

      int size() const
      {
        return 6;
      }

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double get(int i) const
      {
        if(i <= 2)
          return as_location_vec().get(i);
        else
          return as_rotation_vec().get(i - 3);
      }

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double set(int i, double val)
      {
        if(i <= 2)
          return as_location_vec().set(i, val);
        else
          return as_rotation_vec().set(i - 3, val);
      }

      typedef Pose_Vector Base_Type;

      Base_Type &as_vec()
      {
        return static_cast<Base_Type &>(*this);
      }

      const Base_Type &as_vec() const
      {
        return static_cast<const Base_Type &>(*this);
      }

      Location_Vector &as_location_vec()
      {
        return static_cast<Location_Vector &>(*this);
      }

      const Location_Vector &as_location_vec() const
      {
        return static_cast<const Location_Vector &>(*this);
      }

      Rotation_Vector &as_rotation_vec()
      {
        return static_cast<Rotation_Vector &>(*this);
      }

      const Rotation_Vector &as_rotation_vec() const
      {
        return static_cast<const Rotation_Vector &>(*this);
      }
    };

    inline std::ostream &operator<<(std::ostream &o, const Pose_Vector &pose)
    {
      o << "(" << pose.as_location_vec() << "," << pose.as_rotation_vec() << ")";
      return o;
    }

    /**
     * Represents a combination of Location and Rotation within a single
     * reference frame.
     **/
    class Pose : public Pose_Vector, public Coordinate<Pose>
    {
    public:
      Pose(double x, double y, double z, double rx, double ry, double rz)
        : Pose_Vector(x, y, z, rx, ry, rz), Coordinate() {}

      Pose(double x, double y, double z = 0.0)
        : Pose_Vector(x, y, z, 0, 0, 0), Coordinate() {}

      Pose(const Reference_Frame &frame, double x, double y, double z, double rx, double ry, double rz)
        : Pose_Vector(x, y, z, rx, ry, rz), Coordinate(frame) {}

      Pose(const Reference_Frame &frame, double x, double y, double z = 0.0)
        : Pose_Vector(x, y, z, 0, 0, 0), Coordinate(frame) {}

      Pose() : Pose_Vector(), Coordinate() {}

      Pose(const Location &loc)
        : Pose_Vector(loc), Coordinate(loc.frame()) {}

      Pose(const Rotation &rot)
        : Pose_Vector(rot), Coordinate(rot.frame()) {}

      Pose(const Reference_Frame &frame, const Location_Vector &loc, const Rotation_Vector &rot)
        : Pose_Vector(loc, rot), Coordinate(frame) {}

      /// Precondition: loc.frame == rot.frame
      Pose(const Location &loc, const Rotation &rot)
        : Pose_Vector(loc, rot), Coordinate(loc.frame()) {}

      Pose(const Reference_Frame &new_frame, const Rotation &orig)
        : Pose_Vector(orig), Coordinate(orig.frame())
      {
        transform_this_to(new_frame);
      }

      double angle_to(const Pose &target) const
      {
        Rotation me(*this);
        Rotation other(target);
        return me.distance_to(other);
      }

      double angle_to(const Rotation &target) const
      {
        Rotation me(*this);
        return me.distance_to(target);
      }

      operator Location() const
      {
        return Location(frame(), x(), y(), z());
      }

      operator Rotation() const
      {
        return Rotation(frame(), rx(), ry(), rz());
      }

      using Coordinate<Pose>::operator==;
    };
  }
}

// Include if not already included
#include <gams/utility/Reference_Frame.h>

#endif
