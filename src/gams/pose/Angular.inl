/**
 * Copyright (c) 2018 Carnegie Mellon University. All Rights Reserved.
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
 * @file Angular.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains inline functions for the Angular
 **/

#ifndef _GAMS_POSE_ANGULAR_INL_
#define _GAMS_POSE_ANGULAR_INL_

#include <stdexcept>

#include "Angular.h"

namespace gams
{
  namespace pose
  {
    inline AngularVector::AngularVector(
            double rx, double ry, double rz)
      : rx_(rx), ry_(ry), rz_(rz) {}

    inline AngularVector::AngularVector(
            double x, double y, double z, double angle)
      : rx_(x * DEG_TO_RAD(angle)),
        ry_(y * DEG_TO_RAD(angle)),
        rz_(z * DEG_TO_RAD(angle)) {}

    inline AngularVector::AngularVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : rx_(vec[0]), ry_(vec[1]), rz_(vec[2]) {}

    inline AngularVector::AngularVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : rx_(vec[0]), ry_(vec[1]), rz_(vec[2]) {}

    inline AngularVector::AngularVector()
      : rx_(0), ry_(0), rz_(0) {}

    inline bool AngularVector::is_set () const
    {
      return rx_ != INVAL_COORD || ry_ != INVAL_COORD || rz_ != INVAL_COORD;
    }

    inline bool AngularVector::is_zero() const
    {
      return rx_ == 0 && ry_ == 0 && rz_ == 0;
    }

    inline bool AngularVector::operator==(
            const AngularVector &other) const
    {
      return rx_ == other.rx_ && ry_ == other.ry_ && rz_ == other.rz_;
    }

    inline std::string AngularVector::name()
    {
      return "Angular";
    }

    inline double AngularVector::rx() const { return rx_; }
    inline double AngularVector::ry() const { return ry_; }
    inline double AngularVector::rz() const { return rz_; }

    inline double AngularVector::rx(double new_rx) { return rx_ = new_rx; }
    inline double AngularVector::ry(double new_ry) { return ry_ = new_ry; }
    inline double AngularVector::rz(double new_rz) { return rz_ = new_rz; }

    inline AngularVector &AngularVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline const AngularVector &AngularVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline int AngularVector::size() const
    {
      return 3;
    }

    inline double AngularVector::get(int i) const
    {
      return i == 0 ? rx() :
             i == 1 ? ry() :
             i == 2 ? rz() :
            throw std::range_error("Index out of bounds for Angular");
    }

    inline double AngularVector::set(int i, double val)
    {
      if(i == 0)
        return rx(val);
      else if(i == 1)
        return ry(val);
      else if(i == 2)
        return rz(val);
      throw std::range_error("Index out of bounds for Angular");
    }

    inline std::ostream &operator<<(std::ostream &o, const AngularVector &rot)
    {
      o << "(" << rot.rx() << "," << rot.ry() << "," << rot.rz() << ")";
      return o;
    }

    template<class C>
    inline std::string Angular<C>::to_string (const std::string & delimiter,
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

    template<class C>
    inline Angular<C>::Angular(double rx, double ry, double rz)
      : AngularVector(rx, ry, rz),
        Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(
            ReferenceFrame frame, double rx, double ry, double rz)
      : AngularVector(rx, ry, rz),
        Coordinate<C>(frame) {}

    template<class C>
    inline Angular<C>::Angular(double x, double y, double z, double angle)
      : AngularVector(x, y, z, angle), Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(
       ReferenceFrame frame, double x, double y, double z, double angle)
      : AngularVector(x, y, z, angle), Coordinate<C>(frame) {}

    template<class C>
    template<typename U>
    inline Angular<C>::Angular(double rx, double ry, double rz, U u)
      : AngularVector(u.to_radians(rx), u.to_radians(ry), u.to_radians(rz)),
        Coordinate<C>() {}

    template<class C>
    template<typename U>
    inline Angular<C>::Angular(
            ReferenceFrame frame, double rx, double ry, double rz, U u)
      : AngularVector(u.to_radians(rx), u.to_radians(ry), u.to_radians(rz)),
        Coordinate<C>(frame) {}

    template<class C>
    template<typename U>
    inline Angular<C>::Angular(double x, double y, double z, double angle, U u)
      : AngularVector(x, y, z, u.to_radians(angle)), Coordinate<C>() {}

    template<class C>
    template<typename U>
    inline Angular<C>::Angular(
       ReferenceFrame frame, double x, double y, double z,
                                    double angle, U u)
      : AngularVector(x, y, z, u.to_radians(angle)), Coordinate<C>(frame) {}

    template<class C>
    inline Angular<C>::Angular() : AngularVector(), Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(const Quaternion &quat)
      : AngularVector(quat), Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(
          ReferenceFrame frame, const Quaternion &quat)
      : AngularVector(quat), Coordinate<C>(frame) {}

    template<class C>
    inline Angular<C>::Angular(
                ReferenceFrame new_frame, const C &orig)
      : AngularVector(orig), Coordinate<C>(orig.frame())
    {
      this->transform_this_to(new_frame);
    }

    template<class C>
    inline Angular<C>::Angular(
        const madara::knowledge::containers::DoubleVector &vec)
      : AngularVector(vec), Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(ReferenceFrame frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : AngularVector(vec), Coordinate<C>(frame) {}

    template<class C>
    inline Angular<C>::Angular(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : AngularVector(vec), Coordinate<C>() {}

    template<class C>
    inline Angular<C>::Angular(
        ReferenceFrame frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : AngularVector(vec), Coordinate<C>(frame) {}

    template<class C>
    inline double Angular<C>::angle_to(const C &target) const
    {
      return distance_to(target);
    }

    template<class C>
    template<typename U>
    inline double Angular<C>::angle_to (const C &target, U u) const
    {
      return u.from_radians (this->distance_to (target));
    }

    template<class C>
    void Angular<C>::to_container (
      madara::knowledge::containers::NativeDoubleVector &container) const
    {
      if (this->frame ().name () == "GPS")
      {
        container.set (0, get (order::GPS::find (0)));
        container.set (1, get (order::GPS::find (1)));
        container.set (2, get (order::GPS::find (2)));
      }
      else
      {
        container.set (0, get (order::XYZ::find (0)));
        container.set (1, get (order::XYZ::find (1)));
        container.set (2, get (order::XYZ::find (2)));
      }
    }


    template<class C>
    void Angular<C>::from_container (
      const madara::knowledge::containers::NativeDoubleVector &container)
    {
      if (this->frame ().name () == "GPS")
      {
        set (0, container[order::GPS::get (0)]);
        set (1, container[order::GPS::get (1)]);
        set (2, container[order::GPS::get (2)]);
      }
      else
      {
        set (0, container[order::XYZ::get (0)]);
        set (1, container[order::XYZ::get (1)]);
        set (2, container[order::XYZ::get (2)]);
      }
    }
  }
}

#endif
