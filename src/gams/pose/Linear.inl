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
 * @file Linear.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the inline functions for the Linear class
 **/

#ifndef _GAMS_POSE_LINEAR_INL_
#define _GAMS_POSE_LINEAR_INL_

#include "Linear.h"

namespace gams
{
  namespace pose
  {
    inline LinearVector::LinearVector(
                            double x, double y, double z)
      : v_{x, y, z} {}

    inline LinearVector::LinearVector()
      : v_{0, 0, 0} {}

    inline LinearVector::LinearVector(const double array[])
      : v_{array[0], array[1], array[2]} {}

    inline LinearVector::LinearVector(const float array[])
      : v_{array[0], array[1], array[2]} {}

    inline LinearVector::LinearVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : v_{vec[0], vec[1], vec[2]} {}

    inline LinearVector::LinearVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : v_{vec[0], vec[1], vec[2]} {}

    inline bool LinearVector::is_set() const
    {
      return x() != INVAL_COORD || y() != INVAL_COORD || z() != INVAL_COORD;
    }

    inline bool
        LinearVector::operator==(const LinearVector &other) const
    {
      return x() == other.x() && y() == other.y() && z() == other.z();
    }

    inline bool LinearVector::is_zero() const
    {
      return x() == 0 && y() == 0 && z() == 0;
    }

    inline std::string LinearVector::name()
    {
      return "Linear";
    }

    inline double LinearVector::x() const { return v_[0]; }
    inline double LinearVector::y() const { return v_[1]; }
    inline double LinearVector::z() const { return v_[2]; }

    inline double LinearVector::x(double in) { return v_[0] = in; }
    inline double LinearVector::y(double in) { return v_[1] = in; }
    inline double LinearVector::z(double in) { return v_[2] = in; }

    inline double LinearVector::lat() const { return x(); }
    inline double LinearVector::latitude() const { return x(); }
    inline double LinearVector::lon() const { return y(); }
    inline double LinearVector::lng() const { return y(); }
    inline double LinearVector::longitude() const { return y(); }
    inline double LinearVector::alt() const { return -z(); }
    inline double LinearVector::altitude() const { return -z(); }

    inline double LinearVector::lat(double in) { return x(in); }
    inline double LinearVector::latitude(double in) { return x(in); }
    inline double LinearVector::lon(double in) { return y(in); }
    inline double LinearVector::lng(double in) { return y(in); }
    inline double LinearVector::longitude(double in) { return y(in); }
    inline double LinearVector::alt(double in) { return z(in); }
    inline double LinearVector::altitude(double in) { return z(in); }

    inline double LinearVector::rho() const { return x(); }
    inline double LinearVector::theta() const { return x(); }
    inline double LinearVector::phi() const { return y(); }
    inline double LinearVector::r() const { return z(); }

    inline double LinearVector::rho(double in) { return x(in); }
    inline double LinearVector::theta(double in) { return x(in); }
    inline double LinearVector::phi(double in) { return y(in); }
    inline double LinearVector::r(double in) { return z(in); }

    inline int LinearVector::size()
    {
      return 3;
    }

    inline double LinearVector::get(size_t i) const
    {
      if (i >= v_.size()) {
       throw std::range_error("Index out of bounds for Linear");
      }
      return v_[i];
    }

    inline double LinearVector::set(size_t i, double val)
    {
      if (i >= v_.size()) {
       throw std::range_error("Index out of bounds for Linear");
      }
      return v_[i] = val;
    }

    inline LinearVector &LinearVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline const LinearVector &LinearVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const LinearVector &loc)
    {
      o << "(" << loc.x() << "," << loc.y() << "," << loc.z() << ")";
      return o;
    }

    template<class C>
    inline std::string Linear<C>::to_string (const std::string & delimiter,
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

    template<class C>
    inline Linear<C>::Linear(double x, double y, double z)
      : LinearVector(x, y, z), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame frame,
                       double x, double y, double z)
      : LinearVector(x, y, z), Coordinate<C>(frame) {}

    template<class C>
    inline Linear<C>::Linear() : LinearVector(), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame frame)
      : LinearVector(), Coordinate<C>(frame) {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame new_frame,
                              const C &orig)
      : LinearVector(orig), Coordinate<C>(orig.frame())
    {
      this->transform_this_to(new_frame);
    }

    template<class C>
    inline Linear<C>::Linear(const double array[])
      : LinearVector(array), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame frame, const double array[])
      : LinearVector(array), Coordinate<C>(frame) {}

    template<class C>
    inline Linear<C>::Linear(const float array[])
      : LinearVector(array), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame frame, const float array[])
      : LinearVector(array), Coordinate<C>(frame) {}

    template<class C>
    inline Linear<C>::Linear(
        const madara::knowledge::containers::DoubleVector &vec)
      : LinearVector(vec), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(ReferenceFrame frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : LinearVector(vec), Coordinate<C>(frame) {}

    template<class C>
    inline Linear<C>::Linear(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : LinearVector(vec), Coordinate<C>() {}

    template<class C>
    inline Linear<C>::Linear(
        ReferenceFrame frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : LinearVector(vec), Coordinate<C>(frame) {}

    template<class C>
    inline void Linear<C>::to_container (
      madara::knowledge::containers::NativeDoubleVector &container) const
    {
      container.set (0, get (0));
      container.set (1, get (1));
      container.set (2, get (2));
    }


    template<class C>
    inline void Linear<C>::from_container (
      const madara::knowledge::containers::NativeDoubleVector &container)
    {
      set (0, container[0]);
      set (1, container[1]);
      set (2, container[2]);
    }
  }
}

#endif
