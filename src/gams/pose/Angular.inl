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
      : rv_{{rx, ry, rz}} {}

    inline AngularVector::AngularVector(
            double x, double y, double z, double angle)
      : rv_{{x * DEG_TO_RAD(angle),
             y * DEG_TO_RAD(angle),
             z * DEG_TO_RAD(angle)}} {}

    inline AngularVector::AngularVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : rv_{{vec[0], vec[1], vec[2]}} {}

    inline AngularVector::AngularVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : rv_{{vec[0], vec[1], vec[2]}} {}

    inline AngularVector::AngularVector()
      : rv_{{0, 0, 0}} {}

    inline bool AngularVector::is_set () const
    {
      return rx() != INVAL_COORD || ry() != INVAL_COORD || rz() != INVAL_COORD;
    }

    inline bool AngularVector::is_zero() const
    {
      return rx() == 0 && ry() == 0 && rz() == 0;
    }

    inline bool AngularVector::operator==(
            const AngularVector &other) const
    {
      return rx() == other.rx() && ry() == other.ry() && rz() == other.rz();
    }

    inline std::string AngularVector::name()
    {
      return "Angular";
    }

    inline double AngularVector::rx() const { return rv_[0]; }
    inline double AngularVector::ry() const { return rv_[1]; }
    inline double AngularVector::rz() const { return rv_[2]; }

    inline double AngularVector::rx(double in) { return rv_[0] = in; }
    inline double AngularVector::ry(double in) { return rv_[1] = in; }
    inline double AngularVector::rz(double in) { return rv_[2] = in; }

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

    inline double AngularVector::get(size_t i) const
    {
      if (i >= rv_.size()) {
       throw std::range_error("Index out of bounds for Linear");
      }
      return rv_[i];
    }

    inline double AngularVector::set(size_t i, double val)
    {
      if (i >= rv_.size()) {
       throw std::range_error("Index out of bounds for Linear");
      }
      return rv_[i] = val;
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
      return this->distance_to(target);
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
      container.set (0, get (0));
      container.set (1, get (1));
      container.set (2, get (2));
    }

    template<class C>
    void Angular<C>::from_container (
      const madara::knowledge::containers::NativeDoubleVector &container)
    {
      set (0, container[0]);
      set (1, container[1]);
      set (2, container[2]);
    }
  }
}

#endif
