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

#ifndef _GAMS_UTILITY_ROTATION_INL_
#define _GAMS_UTILITY_ROTATION_INL_

namespace gams
{
  namespace utility
  {
    inline constexpr Rotation_Vector::Rotation_Vector(
            double rx, double ry, double rz)
      : rx_(rx), ry_(ry), rz_(rz) {}

    inline constexpr Rotation_Vector::Rotation_Vector(
            double x, double y, double z, double angle)
      : rx_(x * angle), ry_(y * angle), rz_(z * angle) {}

    inline constexpr Rotation_Vector::Rotation_Vector(
            int axis_index, double angle)
      : rx_(axis_index == X_axis ? DEG_TO_RAD(angle) : 0),
        ry_(axis_index == Y_axis ? DEG_TO_RAD(angle) : 0),
        rz_(axis_index == Z_axis ? DEG_TO_RAD(angle) : 0) {}

    inline constexpr Rotation_Vector::Rotation_Vector()
      : rx_(INVAL_COORD), ry_(INVAL_COORD), rz_(INVAL_COORD) {}

    inline constexpr Rotation_Vector::Rotation_Vector(
            const Rotation_Vector &orig)
      : rx_(orig.rx_), ry_(orig.ry_), rz_(orig.rz_) {}

    inline constexpr bool Rotation_Vector::is_invalid() const
    {
      return rx_ == INVAL_COORD || ry_ == INVAL_COORD || rz_ == INVAL_COORD;
    }

    inline constexpr bool Rotation_Vector::is_zero() const
    {
      return rx_ == 0 && ry_ == 0 && rz_ == 0;
    }

    inline constexpr bool Rotation_Vector::operator==(
            const Rotation_Vector &other) const
    {
      return rx_ == other.rx_ && ry_ == other.ry_ && rz_ == other.rz_;
    }

    inline std::string Rotation_Vector::name()
    {
      return "Rotation";
    }

    inline constexpr double Rotation_Vector::rx() const { return rx_; }
    inline constexpr double Rotation_Vector::ry() const { return ry_; }
    inline constexpr double Rotation_Vector::rz() const { return rz_; }

    inline double Rotation_Vector::rx(double new_rx) { return rx_ = new_rx; }
    inline double Rotation_Vector::ry(double new_ry) { return ry_ = new_ry; }
    inline double Rotation_Vector::rz(double new_rz) { return rz_ = new_rz; }

    inline Rotation_Vector &Rotation_Vector::as_vec()
    {
      return static_cast<Base_Type &>(*this);
    }

    inline constexpr const Rotation_Vector &Rotation_Vector::as_vec() const
    {
      return static_cast<const Base_Type &>(*this);
    }

    inline constexpr int Rotation_Vector::size() const
    {
      return 3;
    }

    /**
     * Retrives i'th coordinate, 0-indexed, in order rx, ry, rz
     **/
    inline constexpr double Rotation_Vector::get(int i) const
    {
      return i == 0 ? rx() :
             i == 1 ? ry() :
             i == 2 ? rz() :
            throw std::range_error("Index out of bounds for Rotation");
    }

    /**
     * Sets i'th coordinate, 0-indexed, in order rx, ry, rz
     **/
    inline double Rotation_Vector::set(int i, double val)
    {
      if(i == 0)
        return rx(val);
      else if(i == 1)
        return ry(val);
      else if(i == 2)
        return rz(val);
      throw std::range_error("Index out of bounds for Rotation");
    }

    inline std::ostream &operator<<(std::ostream &o, const Rotation_Vector &rot)
    {
      o << "(" << rot.rx() << "," << rot.ry() << "," << rot.rz() << ")";
      return o;
    }

    inline Rotation::Rotation(double rx, double ry, double rz)
      : Rotation_Vector(rx, ry, rz), Coordinate() {}

    inline constexpr Rotation::Rotation(
            const Reference_Frame &frame, double rx, double ry, double rz)
      : Rotation_Vector(rx, ry, rz), Coordinate(frame) {}

    /**
     * (x, y, z) must be a unit vector in the direction of rotation axis.
     * angle is the angle of rotation, in degrees, about that axis
     **/
    inline Rotation::Rotation(double x, double y, double z, double angle)
      : Rotation_Vector(x, y, z, DEG_TO_RAD(angle)), Coordinate() {}

    inline constexpr Rotation::Rotation(
       const Reference_Frame &frame, double x, double y, double z, double angle)
      : Rotation_Vector(x, y, z, DEG_TO_RAD(angle)), Coordinate(frame) {}

    /**
     * For easy specification of a simple rotation around a single axis,
     * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
     * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
     * around that axis as the angle, in degrees
     **/
    inline Rotation::Rotation(int axis_index, double angle)
      : Rotation_Vector(axis_index, angle), Coordinate() {}

    inline constexpr Rotation::Rotation(
          const Reference_Frame &frame, int axis_index, double angle)
      : Rotation_Vector(axis_index, angle), Coordinate(frame) {}

    inline Rotation::Rotation() : Rotation_Vector(), Coordinate() {}

    inline constexpr Rotation::Rotation(const Rotation &orig)
      : Rotation_Vector(orig), Coordinate(orig) {}

    inline Rotation::Rotation(const Quaternion &quat)
      : Rotation_Vector(quat), Coordinate() {}

    inline Rotation::Rotation(
          const Reference_Frame &frame, const Quaternion &quat)
      : Rotation_Vector(quat), Coordinate(frame) {}

    inline Rotation::Rotation(
                const Reference_Frame &new_frame, const Rotation &orig)
      : Rotation_Vector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline double Rotation::angle_to(const Rotation &target) const
    {
      return distance_to(target);
    }
  }
}

#endif
