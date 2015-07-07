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
 * @file Quaternion.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains inline function definitions for Quaternion
 **/

#ifndef _GAMS_UTILITY_QUATERNION_INL_
#define _GAMS_UTILITY_QUATERNION_INL_

#include <gams/utility/Rotation.h>
#include <gams/utility/Location.h>

namespace gams
{
  namespace utility
  {
    inline constexpr Quaternion::Quaternion(
              double x, double y, double z, double w)
      : x_(x), y_(y), z_(z), w_(w) {}

    inline Quaternion::Quaternion(double rx, double ry, double rz)
    {
      from_rotation_vector(rx, ry, rz);
    }

    inline Quaternion::Quaternion(const Rotation_Vector &rot)
    {
      from_rotation_vector(rot);
    }

    inline Quaternion::Quaternion(const Location_Vector &loc)
    {
      from_location_vector(loc);
    }

    inline void Quaternion::from_location_vector(double x, double y, double z)
    {
      x_ = x;
      y_ = y;
      z_ = z;
      w_ = 0;
    }

    inline void Quaternion::from_location_vector(const Location_Vector &loc)
    {
      from_location_vector(loc.x(), loc.y(), loc.z());
    }

    inline void Quaternion::to_location_vector(
                        double &x, double &y, double &z) const
    {
      x = x_;
      y = y_;
      z = z_;
    }

    inline void Quaternion::to_location_vector(Location_Vector &loc) const
    {
      to_location_vector(loc.x_, loc.y_, loc.z_);
    }

    inline void Quaternion::from_rotation_vector(
                          double rx, double ry, double rz)
    {
      double magnitude = sqrt(rx * rx + ry * ry + rz * rz);
      if(magnitude == 0)
      {
        w_ = 1;
        x_ = y_ = z_ = 0;
      }
      else
      {
        double half_mag = magnitude / 2;
        double cos_half_mag = cos(half_mag);
        double sin_half_mag = sin(half_mag);
        w_ = cos_half_mag;
        x_ = (rx / magnitude) * sin_half_mag;
        y_ = (ry / magnitude) * sin_half_mag;
        z_ = (rz / magnitude) * sin_half_mag;
      }
    }

    inline void Quaternion::from_rotation_vector(const Rotation_Vector &rot)
    {
      from_rotation_vector(rot.rx(), rot.ry(), rot.rz());
    }

    inline void Quaternion::to_rotation_vector(
                              double &rx, double &ry, double &rz) const
    {
      double norm = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
      double angle = 2 * atan2(norm, w_);
      double sin_half_angle = sin(angle / 2);
      if(sin_half_angle < 1e-10)
      {
        rx = ry = rz = 0;
      }
      else
      {
        rx = (x_ / sin_half_angle) * angle;
        ry = (y_ / sin_half_angle) * angle;
        rz = (z_ / sin_half_angle) * angle;
      }
    }

    inline void Quaternion::to_rotation_vector(Rotation_Vector &rot) const
    {
      to_rotation_vector(rot.rx_, rot.ry_, rot.rz_);
    }

    inline void Quaternion::hamilton_product(
                 Quaternion &into, const Quaternion &lhs, const Quaternion &rhs)
    {
      double a = (lhs.w_ + lhs.x_) * (rhs.w_ + rhs.x_),
             b = (lhs.z_ - lhs.y_) * (rhs.y_ - rhs.z_),
             c = (lhs.w_ - lhs.x_) * (rhs.y_ + rhs.z_),
             d = (lhs.y_ + lhs.z_) * (rhs.w_ - rhs.x_),
             e = (lhs.x_ + lhs.z_) * (rhs.x_ + rhs.y_),
             f = (lhs.x_ - lhs.z_) * (rhs.x_ - rhs.y_),
             g = (lhs.w_ + lhs.y_) * (rhs.w_ - rhs.z_),
             h = (lhs.w_ - lhs.y_) * (rhs.w_ + rhs.z_);

      into.w_ = b + (-e - f + g + h) / 2;
      into.x_ = a - ( e + f + g + h) / 2;
      into.y_ = c + ( e - f + g - h) / 2;
      into.z_ = d + ( e - f - g + h) / 2;
    }

    inline Quaternion &Quaternion::operator*=(const Quaternion &rhs)
    {
      hamilton_product(*this, *this, rhs);

      return *this;
    }

    inline Quaternion Quaternion::operator*(const Quaternion &o) const
    {
      Quaternion ret(*this);
      ret *= o;
      return ret;
    }

    inline Quaternion &Quaternion::conjugate()
    {
      x_ = -x_;
      y_ = -y_;
      z_ = -z_;
      return *this;
    }

    inline constexpr Quaternion Quaternion::operator-() const
    {
      return Quaternion(-x_, -y_, -z_, w_);
    }

    inline double Quaternion::inner_product(const Quaternion &o) const
    {
      return w_ * o.w_ + x_ * o.x_ + y_ * o.y_ + z_ * o.z_;
    }

    inline double Quaternion::angle_to(const Quaternion &o) const
    {
      double prod = inner_product(o);
      return acos(2 * prod * prod - 1);
    }

    inline constexpr double Quaternion::x() const { return x_; }
    inline constexpr double Quaternion::y() const { return y_; }
    inline constexpr double Quaternion::z() const { return z_; }
    inline constexpr double Quaternion::w() const { return w_; }

    inline double Quaternion::x(double new_x) { return x_ = new_x; }
    inline double Quaternion::y(double new_y) { return y_ = new_y; }
    inline double Quaternion::z(double new_z) { return z_ = new_z; }
    inline double Quaternion::w(double new_w) { return w_ = new_w; }

    inline Rotation_Vector::Rotation_Vector(const Quaternion &quat)
    {
      quat.to_rotation_vector(rx_, ry_, rz_);
    }

    inline std::ostream &operator<<(std::ostream &o, const Quaternion &quat)
    {
      o << quat.w() << "+" << quat.x() << "i+"
        << quat.y() << "j+" << quat.z() << "z";
      return o;
    }
  }
}

#endif
