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
 * @file Orientation.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains inline functions for the Orientation
 **/

#ifndef _GAMS_UTILITY_ROTATION_INL_
#define _GAMS_UTILITY_ROTATION_INL_

#include <stdexcept>

#include "Orientation.h"

namespace gams
{
  namespace utility
  {
    inline constexpr OrientationVector::OrientationVector(
            double rx, double ry, double rz)
      : rx_(rx), ry_(ry), rz_(rz) {}

    inline constexpr OrientationVector::OrientationVector(
            double x, double y, double z, double angle)
      : rx_(x * DEG_TO_RAD(angle)),
        ry_(y * DEG_TO_RAD(angle)),
        rz_(z * DEG_TO_RAD(angle)) {}

    inline OrientationVector::OrientationVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : rx_(vec[0]), ry_(vec[1]), rz_(vec[2]) {}

    inline OrientationVector::OrientationVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : rx_(vec[0]), ry_(vec[1]), rz_(vec[2]) {}

    inline constexpr OrientationVector::OrientationVector()
      : rx_(INVAL_COORD), ry_(INVAL_COORD), rz_(INVAL_COORD) {}

    inline constexpr bool OrientationVector::is_set () const
    {
      return rx_ != INVAL_COORD || ry_ != INVAL_COORD || rz_ != INVAL_COORD;
    }

    inline constexpr bool OrientationVector::is_zero() const
    {
      return rx_ == 0 && ry_ == 0 && rz_ == 0;
    }

    inline constexpr bool OrientationVector::operator==(
            const OrientationVector &other) const
    {
      return rx_ == other.rx_ && ry_ == other.ry_ && rz_ == other.rz_;
    }

    inline std::string OrientationVector::name()
    {
      return "Orientation";
    }

    inline constexpr double OrientationVector::rx() const { return rx_; }
    inline constexpr double OrientationVector::ry() const { return ry_; }
    inline constexpr double OrientationVector::rz() const { return rz_; }

    inline double OrientationVector::rx(double new_rx) { return rx_ = new_rx; }
    inline double OrientationVector::ry(double new_ry) { return ry_ = new_ry; }
    inline double OrientationVector::rz(double new_rz) { return rz_ = new_rz; }

    inline OrientationVector &OrientationVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline constexpr const OrientationVector &OrientationVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline constexpr int OrientationVector::size() const
    {
      return 3;
    }

    inline constexpr double OrientationVector::get(int i) const
    {
      return i == 0 ? rx() :
             i == 1 ? ry() :
             i == 2 ? rz() :
            throw std::range_error("Index out of bounds for Orientation");
    }

    inline double OrientationVector::set(int i, double val)
    {
      if(i == 0)
        return rx(val);
      else if(i == 1)
        return ry(val);
      else if(i == 2)
        return rz(val);
      throw std::range_error("Index out of bounds for Orientation");
    }

    inline std::ostream &operator<<(std::ostream &o, const OrientationVector &rot)
    {
      o << "(" << rot.rx() << "," << rot.ry() << "," << rot.rz() << ")";
      return o;
    }

    inline std::string Orientation::to_string (const std::string & delimiter,
      const std::string & unset_identifier) const
    {
      std::stringstream buffer;

      if (rx () != DBL_MAX)
        buffer << rx ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (ry () != DBL_MAX)
        buffer << ry ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (rz () != DBL_MAX)
        buffer << rz ();
      else
        buffer << unset_identifier;

      return buffer.str ();
    }

    inline Orientation::Orientation(double rx, double ry, double rz)
      : OrientationVector(rx, ry, rz),
        Coordinate() {}

    inline constexpr Orientation::Orientation(
            const ReferenceFrame &frame, double rx, double ry, double rz)
      : OrientationVector(rx, ry, rz),
        Coordinate(frame) {}

    inline Orientation::Orientation(double x, double y, double z, double angle)
      : OrientationVector(x, y, z, angle), Coordinate() {}

    inline constexpr Orientation::Orientation(
       const ReferenceFrame &frame, double x, double y, double z, double angle)
      : OrientationVector(x, y, z, angle), Coordinate(frame) {}

    template<typename U>
    inline Orientation::Orientation(double rx, double ry, double rz, U u)
      : OrientationVector(u.to_radians(rx), u.to_radians(ry), u.to_radians(rz)),
        Coordinate() {}

    template<typename U>
    inline constexpr Orientation::Orientation(
            const ReferenceFrame &frame, double rx, double ry, double rz, U u)
      : OrientationVector(u.to_radians(rx), u.to_radians(ry), u.to_radians(rz)),
        Coordinate(frame) {}

    template<typename U>
    inline Orientation::Orientation(double x, double y, double z, double angle, U u)
      : OrientationVector(x, y, z, u.to_radians(angle)), Coordinate() {}

    template<typename U>
    inline constexpr Orientation::Orientation(
       const ReferenceFrame &frame, double x, double y, double z,
                                    double angle, U u)
      : OrientationVector(x, y, z, u.to_radians(angle)), Coordinate(frame) {}

    inline Orientation::Orientation() : OrientationVector(), Coordinate() {}

    inline Orientation::Orientation(const Quaternion &quat)
      : OrientationVector(quat), Coordinate() {}

    inline Orientation::Orientation(
          const ReferenceFrame &frame, const Quaternion &quat)
      : OrientationVector(quat), Coordinate(frame) {}

    inline Orientation::Orientation(
                const ReferenceFrame &new_frame, const Orientation &orig)
      : OrientationVector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline Orientation::Orientation(
        const madara::knowledge::containers::DoubleVector &vec)
      : OrientationVector(vec), Coordinate() {}

    inline Orientation::Orientation(const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : OrientationVector(vec), Coordinate(frame) {}

    inline Orientation::Orientation(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : OrientationVector(vec), Coordinate() {}

    inline Orientation::Orientation(
        const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : OrientationVector(vec), Coordinate(frame) {}

    inline double Orientation::angle_to(const Orientation &target) const
    {
      return distance_to(target);
    }

    template<typename U>
    inline double Orientation::angle_to(const Orientation &target, U u) const
    {
      return u.from_radians(distance_to(target));
    }
  }
}

#endif
