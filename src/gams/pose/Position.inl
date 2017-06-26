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
 * @file Position.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the inline functions for the Position class
 **/

#ifndef _GAMS_POSE_POSITION_INL_
#define _GAMS_POSE_POSITION_INL_

#include "Position.h"

namespace gams
{
  namespace pose
  {
    inline constexpr PositionVector::PositionVector(
                            double x, double y, double z)
      : x_(x), y_(y), z_(z) {}

    inline constexpr PositionVector::PositionVector()
      : x_(0), y_(0), z_(0) {}

    inline PositionVector::PositionVector(const double array[])
      : x_(array[0]), y_(array[1]), z_(array[2]) {}

    inline PositionVector::PositionVector(const float array[])
      : x_(array[0]), y_(array[1]), z_(array[2]) {}

    inline PositionVector::PositionVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : x_(vec[0]), y_(vec[1]), z_(vec[2]) {}

    inline PositionVector::PositionVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : x_(vec[0]), y_(vec[1]), z_(vec[2]) {}

    inline constexpr bool PositionVector::is_set() const
    {
      return x_ != INVAL_COORD || y_ != INVAL_COORD || z_ != INVAL_COORD;
    }

    inline constexpr bool
        PositionVector::operator==(const PositionVector &other) const
    {
      return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
    }

    inline constexpr bool PositionVector::is_zero() const
    {
      return x_ == 0 && y_ == 0 && z_ == 0;
    }

    inline std::string PositionVector::name()
    {
      return "Position";
    }

    inline constexpr double PositionVector::x() const { return x_; }
    inline constexpr double PositionVector::y() const { return y_; }
    inline constexpr double PositionVector::z() const { return z_; }

    inline double PositionVector::x(double new_x) { return x_ = new_x; }
    inline double PositionVector::y(double new_y) { return y_ = new_y; }
    inline double PositionVector::z(double new_z) { return z_ = new_z; }

    inline constexpr double PositionVector::lon() const { return x_; }
    inline constexpr double PositionVector::lng() const { return x_; }
    inline constexpr double PositionVector::longitude() const { return x_; }
    inline constexpr double PositionVector::lat() const { return y_; }
    inline constexpr double PositionVector::latitude() const { return y_; }
    inline constexpr double PositionVector::alt() const { return z_; }
    inline constexpr double PositionVector::altitude() const { return z_; }

    inline double PositionVector::lon(double new_x) { return x_ = new_x; }
    inline double PositionVector::lng(double new_x) { return x_ = new_x; }
    inline double PositionVector::longitude(double new_x) { return x_ = new_x; }
    inline double PositionVector::lat(double new_y) { return y_ = new_y; }
    inline double PositionVector::latitude(double new_y) { return y_ = new_y; }
    inline double PositionVector::alt(double new_z) { return z_ = new_z; }
    inline double PositionVector::altitude(double new_z) { return z_ = new_z; }

    inline constexpr double PositionVector::rho() const { return x_; }
    inline constexpr double PositionVector::theta() const { return x_; }
    inline constexpr double PositionVector::phi() const { return y_; }
    inline constexpr double PositionVector::r() const { return z_; }

    inline double PositionVector::rho(double new_x) { return x_ = new_x; }
    inline double PositionVector::theta(double new_x) { return x_ = new_x; }
    inline double PositionVector::phi(double new_y) { return y_ = new_y; }
    inline double PositionVector::r(double new_z) { return z_ = new_z; }

    inline constexpr int PositionVector::size()
    {
      return 3;
    }

    inline constexpr double PositionVector::get(int i) const
    {
      return i == 0 ? x() :
             i == 1 ? y() :
             i == 2 ? z() :
             throw std::range_error("Index out of bounds for Position");
    }

    inline double PositionVector::set(int i, double val)
    {
      return i == 0 ? x(val) :
             i == 1 ? y(val) :
             i == 2 ? z(val) :
             throw std::range_error("Index out of bounds for Position");
    }

    inline PositionVector &PositionVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline constexpr const PositionVector &PositionVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const PositionVector &loc)
    {
      o << "(" << loc.x() << "," << loc.y() << "," << loc.z() << ")";
      return o;
    }

    inline std::string Position::to_string (const std::string & delimiter,
      const std::string & unset_identifier) const
    {
      std::stringstream buffer;

      if (x () != DBL_MAX)
        buffer << x ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (y () != DBL_MAX)
        buffer << y ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (z () != DBL_MAX)
        buffer << z ();
      else
        buffer << unset_identifier;

      return buffer.str ();
    }

    inline Position::Position(double x, double y, double z)
      : PositionVector(x, y, z), Coordinate() {}

    inline constexpr Position::Position(const ReferenceFrame &frame,
                       double x, double y, double z)
      : PositionVector(x, y, z), Coordinate(frame) {}

    inline Position::Position() : PositionVector(), Coordinate() {}

    inline Position::Position(const ReferenceFrame &frame)
      : PositionVector(), Coordinate(frame) {}

    inline Position::Position(const ReferenceFrame &new_frame,
                              const Position &orig)
      : PositionVector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline Position::Position (const Position & rhs)
      : PositionVector (rhs.x (), rhs.y (), rhs.z ()), Coordinate (rhs.frame ())
    {
    }

    inline Position::Position(const double array[])
      : PositionVector(array), Coordinate() {}

    inline Position::Position(const ReferenceFrame &frame, const double array[])
      : PositionVector(array), Coordinate(frame) {}

    inline Position::Position(const float array[])
      : PositionVector(array), Coordinate() {}

    inline Position::Position(const ReferenceFrame &frame, const float array[])
      : PositionVector(array), Coordinate(frame) {}

    inline Position::Position(
        const madara::knowledge::containers::DoubleVector &vec)
      : PositionVector(vec), Coordinate() {}

    inline Position::Position(const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : PositionVector(vec), Coordinate(frame) {}

    inline Position::Position(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : PositionVector(vec), Coordinate() {}

    inline Position::Position(
        const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : PositionVector(vec), Coordinate(frame) {}

  }
}

#endif
