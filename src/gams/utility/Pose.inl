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

#include "Pose.h"

namespace gams
{
  namespace utility
  {
    inline constexpr PoseVector::PoseVector(double x, double y, double z,
                                              double rx, double ry, double rz)
      : LocationVector(x, y, z), RotationVector(rx, ry, rz) {}

    inline constexpr PoseVector::PoseVector(const LocationVector &loc)
      : LocationVector(loc), RotationVector(0, 0, 0) {}

    inline constexpr PoseVector::PoseVector(const RotationVector &rot)
      : LocationVector(0, 0, 0), RotationVector(rot) {}

    inline constexpr PoseVector::PoseVector(const LocationVector &loc,
                                              const RotationVector &rot)
      : LocationVector(loc), RotationVector(rot) {}

    inline constexpr PoseVector::PoseVector()
      : LocationVector(), RotationVector() {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : LocationVector(vec[0], vec[1], vec[2]),
        RotationVector(vec[3], vec[4], vec[5]) {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot)
      : LocationVector(vec_loc),
        RotationVector(vec_rot) {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : LocationVector(vec[0], vec[1], vec[2]),
        RotationVector(vec[3], vec[4], vec[5]) {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot)
      : LocationVector(vec_loc),
        RotationVector(vec_rot) {}

    inline constexpr bool PoseVector::is_invalid() const
    {
      return LocationVector::is_invalid() || RotationVector::is_invalid();
    }

    inline constexpr bool PoseVector::is_location_zero() const
    {
      return as_location_vec().is_zero();
    }

    inline constexpr bool PoseVector::is_rotation_zero() const
    {
      return as_rotation_vec().is_zero();
    }

    inline constexpr bool PoseVector::is_zero() const
    {
      return is_location_zero() && is_rotation_zero();
    }

    inline constexpr bool PoseVector::operator==(
                                const PoseVector &other) const
    {
      return as_location_vec() == other.as_location_vec() &&
             as_rotation_vec() == other.as_rotation_vec();
    }

    inline std::string PoseVector::name()
    {
      return "Pose";
    }

    inline constexpr int PoseVector::size() const
    {
      return as_location_vec().size() + as_rotation_vec().size();
    }

    inline constexpr double PoseVector::get(int i) const
    {
      return i <= 2 ? as_location_vec().get(i) :
                      as_rotation_vec().get(i - 3);
    }

    inline double PoseVector::set(int i, double val)
    {
      if(i <= 2)
        return as_location_vec().set(i, val);
      else
        return as_rotation_vec().set(i - 3, val);
    }

    inline PoseVector &PoseVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline constexpr const PoseVector &PoseVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline LocationVector &PoseVector::as_location_vec()
    {
      return static_cast<LocationVector &>(*this);
    }

    inline constexpr const LocationVector &PoseVector::as_location_vec() const
    {
      return static_cast<const LocationVector &>(*this);
    }

    inline RotationVector &PoseVector::as_rotation_vec()
    {
      return static_cast<RotationVector &>(*this);
    }

    inline constexpr const RotationVector &PoseVector::as_rotation_vec() const
    {
      return static_cast<const RotationVector &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const PoseVector &pose)
    {
      o << "(" << pose.as_location_vec() << ","
               << pose.as_rotation_vec() << ")";
      return o;
    }

    inline Pose::Pose(double x, double y, double z,
                      double rx, double ry, double rz)
      : PoseVector(x, y, z, rx, ry, rz), Coordinate() {}

    inline Pose::Pose(double x, double y, double z)
      : PoseVector(x, y, z, 0, 0, 0), Coordinate() {}

    inline constexpr Pose::Pose(const ReferenceFrame &frame,
                                double x, double y, double z,
                                double rx, double ry, double rz)
      : PoseVector(x, y, z, rx, ry, rz), Coordinate(frame) {}

    inline constexpr Pose::Pose(const ReferenceFrame &frame,
                                double x, double y, double z)
      : PoseVector(x, y, z, 0, 0, 0), Coordinate(frame) {}

    inline Pose::Pose() : PoseVector(), Coordinate() {}

    inline constexpr Pose::Pose(const Location &loc)
      : PoseVector(loc), Coordinate(loc.frame()) {}

    inline constexpr Pose::Pose(const Rotation &rot)
      : PoseVector(rot), Coordinate(rot.frame()) {}

    inline Pose::Pose(const LocationVector &loc, const RotationVector &rot)
      : PoseVector(loc, rot), Coordinate() {}

    inline constexpr Pose::Pose(const ReferenceFrame &frame,
                      const LocationVector &loc, const RotationVector &rot)
      : PoseVector(loc, rot), Coordinate(frame) {}

    inline constexpr Pose::Pose(const Location &loc, const Rotation &rot)
      : PoseVector(loc, rot), Coordinate(loc.frame()) {}

    inline Pose::Pose(const ReferenceFrame &new_frame, const Pose &orig)
      : PoseVector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline Pose::Pose(
        const madara::knowledge::containers::DoubleVector &vec)
      : PoseVector(vec), Coordinate() {}

    inline Pose::Pose(const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : PoseVector(vec), Coordinate(frame) {}

    inline Pose::Pose(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : PoseVector(vec), Coordinate() {}

    inline Pose::Pose(
        const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : PoseVector(vec), Coordinate(frame) {}

    inline Pose::Pose(
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot)
      : PoseVector(vec_loc, vec_rot), Coordinate() {}

    inline Pose::Pose(const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot)
      : PoseVector(vec_loc, vec_rot), Coordinate(frame) {}

    inline Pose::Pose(
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot)
      : PoseVector(vec_loc, vec_rot), Coordinate() {}

    inline Pose::Pose(
        const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot)
      : PoseVector(vec_loc, vec_rot), Coordinate(frame) {}

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
