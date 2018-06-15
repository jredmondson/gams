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

namespace gams { namespace pose {

template<typename Derived>
inline BasicPose<Derived>::BasicPose(double x, double y, double z,
                              double rx, double ry, double rz)
  : PositionVector(x, y, z), OrientationVector(rx, ry, rz) {}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(const PositionVector &loc)
  : PositionVector (loc), OrientationVector (0, 0, 0)
{
}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(const OrientationVector &rot)
  : PositionVector (0, 0, 0), OrientationVector (rot)
{
}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(const PositionVector &loc,
                              const OrientationVector &rot)
  : PositionVector(loc), OrientationVector(rot) {}

template<typename Derived>
inline BasicPose<Derived>::BasicPose()
  : PositionVector(), OrientationVector() {}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(
    const madara::knowledge::containers::DoubleVector &vec)
  : PositionVector(vec[0], vec[1], vec[2]),
    OrientationVector(vec[3], vec[4], vec[5]) {}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(
    const madara::knowledge::containers::DoubleVector &vec_loc,
    const madara::knowledge::containers::DoubleVector &vec_rot)
  : PositionVector(vec_loc),
    OrientationVector(vec_rot) {}

template<typename Derived>
inline BasicPose<Derived>::BasicPose(
    const madara::knowledge::containers::NativeDoubleVector &vec)
    : PositionVector (vec[0], vec[1], vec[2]),
    OrientationVector (vec[3], vec[4], vec[5])
{
}


template<typename Derived>
inline BasicPose<Derived>::BasicPose(
    const madara::knowledge::containers::NativeDoubleVector &vec_loc,
    const madara::knowledge::containers::NativeDoubleVector &vec_rot)
  : PositionVector(vec_loc),
    OrientationVector(vec_rot) {}

template<typename Derived>
inline bool BasicPose<Derived>::is_set() const
{
  return PositionVector::is_set() || OrientationVector::is_set();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_position_set () const
{
  return PositionVector::is_set ();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_location_set () const
{
  return is_position_set ();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_orientation_set () const
{
  return OrientationVector::is_set ();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_position_zero() const
{
  return as_position_vec().is_zero();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_location_zero() const
{
  return is_position_zero ();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_orientation_zero() const
{
  return as_orientation_vec().is_zero();
}

template<typename Derived>
inline bool BasicPose<Derived>::is_zero() const
{
  return is_position_zero() && is_orientation_zero();
}

template<typename Derived>
inline bool BasicPose<Derived>::operator==(
                            const BasicPose &other) const
{
  return as_position_vec() == other.as_position_vec() &&
         as_orientation_vec() == other.as_orientation_vec();
}

template<typename Derived>
inline std::string BasicPose<Derived>::name()
{
  return "Pose";
}

template<typename Derived>
inline int BasicPose<Derived>::size() const
{
  return as_position_vec().size() + as_orientation_vec().size();
}

template<typename Derived>
inline double BasicPose<Derived>::get(int i) const
{
  return i <= 2 ? as_position_vec().get(i) :
                  as_orientation_vec().get(i - 3);
}

template<typename Derived>
inline double BasicPose<Derived>::set(int i, double val)
{
  if(i <= 2)
    return as_position_vec().set(i, val);
  else
    return as_orientation_vec().set(i - 3, val);
}

template<typename Derived>
inline BasicPose<Derived> &BasicPose<Derived>::as_vec()
{
  return static_cast<BasicPose &>(*this);
}

template<typename Derived>
inline const BasicPose<Derived> &BasicPose<Derived>::as_vec() const
{
  return static_cast<const BasicPose &>(*this);
}

template<typename Derived>
inline PositionVector &BasicPose<Derived>::as_position_vec()
{
  return static_cast<PositionVector &>(*this);
}

template<typename Derived>
inline const PositionVector &BasicPose<Derived>::as_position_vec() const
{
  return static_cast<const PositionVector &>(*this);
}

template<typename Derived>
inline PositionVector &BasicPose<Derived>::as_location_vec()
{
  return as_position_vec ();
}

template<typename Derived>
inline const PositionVector &BasicPose<Derived>::as_location_vec() const
{
  return as_position_vec ();
}

template<typename Derived>
inline OrientationVector &BasicPose<Derived>::as_orientation_vec()
{
  return static_cast<OrientationVector &>(*this);
}

template<typename Derived>
inline const OrientationVector &BasicPose<Derived>::as_orientation_vec() const
{
  return static_cast<const OrientationVector &>(*this);
}

inline double Pose::angle_to(const Pose &target) const
{
  Orientation me(*this);
  Orientation other(target);
  return me.distance_to(other);
}

inline double Pose::angle_to(const Orientation &target) const
{
  Orientation me(*this);
  return me.distance_to(target);
}

template<typename U>
inline double Pose::angle_to (const Pose &target, U u) const
{
  Orientation me(*this);
  Orientation other(target);
  return me.angle_to(other, u);
}

template<typename U>
inline double Pose::angle_to (const Orientation &target, U u) const
{
  Orientation me(*this);
  return me.angle_to(target, u);
}

inline Pose::operator Position() const
{
  return Position(frame(), x(), y(), z());
}

inline Pose::operator Orientation() const
{
  return Orientation(frame(), rx(), ry(), rz());
}

template<typename Derived>
inline std::string BasicPose<Derived>::to_string (
    const std::string & delimiter,
    const std::string & unset_identifier) const
{
  std::stringstream buffer;

  buffer << Position (*this).to_string (delimiter, unset_identifier);

  buffer << delimiter;

  buffer << Orientation
  (*this).to_string (delimiter, unset_identifier);

  return buffer.str ();
}

template<typename Derived>
inline void BasicPose<Derived>::to_container (
  madara::knowledge::containers::NativeDoubleVector &container) const
{
  container.resize (6);
  for (int i = 0; i < 6; ++i)
  {
    container.set (i, get (i));
  }
}


template<typename Derived>
inline void BasicPose<Derived>::from_container (
  const madara::knowledge::containers::NativeDoubleVector &container)
{
  for (size_t i = 0; i < 6; ++i)
  {
    if (i < container.size ()) {
      set ((int)i, container[i]);
    } else {
      set ((int)i, 0);
    }
  }
}

template<typename Derived>
inline void BasicPose<Derived>::from_container (
  const std::vector <double> &container)
{
  for (size_t i = 0; i < 6; ++i)
  {
    if (i < container.size ()) {
      set ((int)i, container[i]);
    } else {
      set ((int)i, 0);
    }
  }
}

} }

#endif
