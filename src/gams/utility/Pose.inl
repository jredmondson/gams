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
 * @file Pose.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Pose class's inline function definitions
 **/

#ifndef _GAMS_UTILITY_POSE_INL_
#define _GAMS_UTILITY_POSE_INL_

namespace gams
{
  namespace utility
  {
    inline constexpr Pose_Vector::Pose_Vector(double x, double y, double z,
                                              double rx, double ry, double rz)
      : Location_Vector(x, y, z), Rotation_Vector(rx, ry, rz) {}

    inline constexpr Pose_Vector::Pose_Vector(const Location_Vector &loc)
      : Location_Vector(loc), Rotation_Vector(0, 0, 0) {}

    inline constexpr Pose_Vector::Pose_Vector(const Rotation_Vector &rot)
      : Location_Vector(0, 0, 0), Rotation_Vector(rot) {}

    inline constexpr Pose_Vector::Pose_Vector(const Location_Vector &loc,
                                              const Rotation_Vector &rot)
      : Location_Vector(loc), Rotation_Vector(rot) {}

    inline constexpr Pose_Vector::Pose_Vector()
      : Location_Vector(), Rotation_Vector() {}

    inline constexpr Pose_Vector::Pose_Vector(const Pose_Vector &other)
      : Location_Vector(other.as_location_vec()),
        Rotation_Vector(other.as_rotation_vec()) {}

    inline constexpr bool Pose_Vector::is_invalid() const
    {
      return Location_Vector::is_invalid() || Rotation_Vector::is_invalid();
    }

    inline constexpr bool Pose_Vector::is_location_zero() const
    {
      return as_location_vec().is_zero();
    }

    inline constexpr bool Pose_Vector::is_rotation_zero() const
    {
      return as_rotation_vec().is_zero();
    }

    inline constexpr bool Pose_Vector::is_zero() const
    {
      return is_location_zero() && is_rotation_zero();
    }

    inline constexpr bool Pose_Vector::operator==(
                                const Pose_Vector &other) const
    {
      return as_location_vec() == other.as_location_vec() &&
             as_rotation_vec() == other.as_rotation_vec();
    }

    inline std::string Pose_Vector::name()
    {
      return "Pose";
    }

    inline constexpr int Pose_Vector::size() const
    {
      return as_location_vec().size() + as_rotation_vec().size();
    }

    inline constexpr double Pose_Vector::get(int i) const
    {
      return i <= 2 ? as_location_vec().get(i) :
                      as_rotation_vec().get(i - 3);
    }

    inline double Pose_Vector::set(int i, double val)
    {
      if(i <= 2)
        return as_location_vec().set(i, val);
      else
        return as_rotation_vec().set(i - 3, val);
    }

    inline Pose_Vector &Pose_Vector::as_vec()
    {
      return static_cast<Base_Type &>(*this);
    }

    inline constexpr const Pose_Vector &Pose_Vector::as_vec() const
    {
      return static_cast<const Base_Type &>(*this);
    }

    inline Location_Vector &Pose_Vector::as_location_vec()
    {
      return static_cast<Location_Vector &>(*this);
    }

    inline constexpr const Location_Vector &Pose_Vector::as_location_vec() const
    {
      return static_cast<const Location_Vector &>(*this);
    }

    inline Rotation_Vector &Pose_Vector::as_rotation_vec()
    {
      return static_cast<Rotation_Vector &>(*this);
    }

    inline constexpr const Rotation_Vector &Pose_Vector::as_rotation_vec() const
    {
      return static_cast<const Rotation_Vector &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const Pose_Vector &pose)
    {
      o << "(" << pose.as_location_vec() << ","
               << pose.as_rotation_vec() << ")";
      return o;
    }

    inline Pose::Pose(double x, double y, double z,
                      double rx, double ry, double rz)
      : Pose_Vector(x, y, z, rx, ry, rz), Coordinate() {}

    inline Pose::Pose(double x, double y, double z)
      : Pose_Vector(x, y, z, 0, 0, 0), Coordinate() {}

    inline constexpr Pose::Pose(const Reference_Frame &frame,
                                double x, double y, double z,
                                double rx, double ry, double rz)
      : Pose_Vector(x, y, z, rx, ry, rz), Coordinate(frame) {}

    inline constexpr Pose::Pose(const Reference_Frame &frame,
                                double x, double y, double z)
      : Pose_Vector(x, y, z, 0, 0, 0), Coordinate(frame) {}

    inline Pose::Pose() : Pose_Vector(), Coordinate() {}

    inline constexpr Pose::Pose(const Pose &other)
      : Pose_Vector(other.as_vec()), Coordinate(other.frame()) {}

    inline constexpr Pose::Pose(const Location &loc)
      : Pose_Vector(loc), Coordinate(loc.frame()) {}

    inline constexpr Pose::Pose(const Rotation &rot)
      : Pose_Vector(rot), Coordinate(rot.frame()) {}

    inline Pose::Pose(const Location_Vector &loc, const Rotation_Vector &rot)
      : Pose_Vector(loc, rot), Coordinate() {}

    inline constexpr Pose::Pose(const Reference_Frame &frame,
                      const Location_Vector &loc, const Rotation_Vector &rot)
      : Pose_Vector(loc, rot), Coordinate(frame) {}

    inline constexpr Pose::Pose(const Location &loc, const Rotation &rot)
      : Pose_Vector(loc, rot), Coordinate(loc.frame()) {}

    inline Pose::Pose(const Reference_Frame &new_frame, const Pose &orig)
      : Pose_Vector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline double Pose::angle_to(const Pose &target) const
    {
      Rotation me(*this);
      Rotation other(target);
      return me.distance_to(other);
    }

    inline double Pose::angle_to(const Rotation &target) const
    {
      Rotation me(*this);
      return me.distance_to(target);
    }

    inline constexpr Pose::operator Location() const
    {
      return Location(frame(), x(), y(), z());
    }

    inline constexpr Pose::operator Rotation() const
    {
      return Rotation(frame(), rx(), ry(), rz());
    }
  }
}

#endif
