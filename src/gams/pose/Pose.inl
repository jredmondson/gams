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

#ifndef _GAMS_POSE_POSE_INL_
#define _GAMS_POSE_POSE_INL_

#include "Pose.h"

#include <sstream>

namespace gams
{
  namespace pose
  {
    inline constexpr PoseVector::PoseVector(double x, double y, double z,
                                              double rx, double ry, double rz)
      : PositionVector(x, y, z), RotationVector(rx, ry, rz) {}

    inline constexpr PoseVector::PoseVector(const PositionVector &loc)
      : PositionVector (loc), RotationVector (0, 0, 0)
    {
    }

    inline constexpr PoseVector::PoseVector(const RotationVector &rot)
      : PositionVector (0, 0, 0), RotationVector (rot)
    {
    }

    inline constexpr PoseVector::PoseVector(const PositionVector &loc,
                                            const RotationVector &rot)
      : PositionVector(loc), RotationVector(rot) {}

    inline constexpr PoseVector::PoseVector()
      : PositionVector(), RotationVector() {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : PositionVector(vec[0], vec[1], vec[2]),
        RotationVector(vec[3], vec[4], vec[5]) {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot)
      : PositionVector(vec_loc),
        RotationVector(vec_rot) {}

    inline PoseVector::PoseVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
        : PositionVector (vec[0], vec[1], vec[2]),
        RotationVector (vec[3], vec[4], vec[5])
    {
    }


    inline PoseVector::PoseVector(
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot)
      : PositionVector(vec_loc),
        RotationVector(vec_rot) {}

    inline constexpr bool PoseVector::is_set() const
    {
      return PositionVector::is_set() || RotationVector::is_set();
    }

    inline constexpr bool PoseVector::is_location_set () const
    {
      return PositionVector::is_set ();
    }

    inline constexpr bool PoseVector::is_orientation_set () const
    {
      return RotationVector::is_set ();
    }

    inline constexpr bool PoseVector::is_location_zero() const
    {
      return as_location_vec().is_zero();
    }

    inline constexpr bool PoseVector::is_orientation_zero() const
    {
      return as_orientation_vec().is_zero();
    }

    inline constexpr bool PoseVector::is_zero() const
    {
      return is_location_zero() && is_orientation_zero();
    }

    inline constexpr bool PoseVector::operator==(
                                const PoseVector &other) const
    {
      return as_location_vec() == other.as_location_vec() &&
             as_orientation_vec() == other.as_orientation_vec();
    }

    inline std::string PoseVector::name()
    {
      return "Pose";
    }

    inline constexpr int PoseVector::size() const
    {
      return as_location_vec().size() + as_orientation_vec().size();
    }

    inline constexpr double PoseVector::get(int i) const
    {
      return i <= 2 ? as_location_vec().get(i) :
                      as_orientation_vec().get(i - 3);
    }

    inline double PoseVector::set(int i, double val)
    {
      if(i <= 2)
        return as_location_vec().set(i, val);
      else
        return as_orientation_vec().set(i - 3, val);
    }

    inline PoseVector &PoseVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline constexpr const PoseVector &PoseVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline PositionVector &PoseVector::as_location_vec()
    {
      return static_cast<PositionVector &>(*this);
    }

    inline constexpr const PositionVector &PoseVector::as_location_vec() const
    {
      return static_cast<const PositionVector &>(*this);
    }

    inline RotationVector &PoseVector::as_orientation_vec()
    {
      return static_cast<RotationVector &>(*this);
    }

    inline constexpr const RotationVector &PoseVector::as_orientation_vec() const
    {
      return static_cast<const RotationVector &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const PoseVector &pose)
    {
      o << "(" << pose.as_location_vec() << ","
               << pose.as_orientation_vec() << ")";
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

    inline constexpr Pose::Pose(const Position &loc)
      : PoseVector(loc), Coordinate(loc.frame()) {}

    inline constexpr Pose::Pose(const Rotation &rot)
      : PoseVector(rot), Coordinate(rot.frame()) {}

    inline Pose::Pose(const PositionVector &loc, const RotationVector &rot)
      : PoseVector(loc, rot), Coordinate() {}

    inline constexpr Pose::Pose(const ReferenceFrame &frame,
                      const PositionVector &loc, const RotationVector &rot)
      : PoseVector(loc, rot), Coordinate(frame) {}

    inline constexpr Pose::Pose(const Position &loc, const Rotation &rot)
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

    inline constexpr Pose::operator Position() const
    {
      return Position(frame(), x(), y(), z());
    }

    inline constexpr Pose::operator Rotation() const
    {
      return Rotation(frame(), rx(), ry(), rz());
    }

    inline
      std::string Pose::to_string (const std::string & delimiter,
      const std::string & unset_identifier) const
    {
      std::stringstream buffer;

      buffer << Position (*this).to_string (delimiter, unset_identifier);

      buffer << delimiter;

      buffer << Rotation (*this).to_string (delimiter, unset_identifier);

      return buffer.str ();
    }


  }
}

#endif
