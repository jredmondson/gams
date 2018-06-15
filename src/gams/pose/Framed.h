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
 * @file Framed.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Contains the Framed template, which adds a bound frame to coordinates
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_FRAMED_H_
#define _GAMS_POSE_FRAMED_H_

#include "gams/GamsExport.h"
#include <string>
#include <cfloat>
#include <utility>
#include <gams/CPP11_compat.h>
#include <madara/knowledge/containers/DoubleVector.h>
#include <madara/knowledge/containers/NativeDoubleVector.h>
#include "ReferenceFrameFwd.h"
#include "Coordinate.h"

namespace gams { namespace pose {

template<typename T>
inline auto try_transform_to(const T& v, const ReferenceFrame &frame) ->
  typename std::enable_if<supports_transform_to<T>::value, T>::type
{
  return v.transform_to(frame);
}

template<typename T>
inline auto try_transform_to(const T& v, const ReferenceFrame &) ->
  typename std::enable_if<!supports_transform_to<T>::value, const T &>::type
{
  return v;
}

inline ReferenceFrame try_get_frame()
{
  return default_frame();
}

template<typename T, typename... Args>
inline auto try_get_frame(const T& v, Args&&...) ->
  typename std::enable_if<supports_transform_to<T>::value, ReferenceFrame>::type
{
  return v.frame();
}

template<typename T, typename... Args>
inline auto try_get_frame(const T&, Args&&... args) ->
  typename std::enable_if<!supports_transform_to<T>::value, ReferenceFrame>::type
{
  return try_get_frame(std::forward<Args>(args)...);
}

/// Internal class implementing coordinates bound to reference frame.
/// Do not use directly.
template<typename Base>
class Framed : public Base
{
public:
  using Base::self;
  using derived_type = typename Base::derived_type;

  /**
   * Default Constructor. Initializes frame as empty
   **/
  Framed() = default;

#ifdef DOXYGEN
  /**
   * Construct without a ReferenceFrame object.
   * Initialized frame as empty, unless at least one argument is a type
   * which includes a frame, in which case that frame will become the frame
   * of this pose. If multiple do, the first such argument will be used.
   *
   * This overload only participates if the first argument isn't ReferenceFrame
   *
   * @param args arguments to pass through to base type
   **/
  template<typename... Args>
  Framed(Args&&... args);
#else
  template<typename Arg, typename... Args,
    typename std::enable_if<!std::is_same<typename std::decay<Arg>::type,
      ReferenceFrame>::value && sizeof...(Args) >= 1, void*>::type = nullptr>
  Framed(Arg&& arg, Args&&... args)
    : Base((Arg &)arg, ((Args &)args)...),
      frame_(try_get_frame((Arg &)arg, ((Args &)args)...)) {}

  template<typename Arg,
    typename std::enable_if<
      !std::is_same<typename std::decay<Arg>::type,
        ReferenceFrame>::value, void*>::type = nullptr>
  explicit Framed(Arg&& arg)
    : Base((Arg &)arg), frame_(try_get_frame(arg)) {}
#endif

  /**
   * Construct using just a ReferenceFrame object. All coordinate values zero.
   *
   * @param frame the ReferenceFrame this Coordinate will belong to
   **/
  explicit Framed(ReferenceFrame frame)
    : Base(), frame_(frame) {}

#ifdef DOXYGEN
  /**
   * Construct using a ReferenceFrame object.
   *
   * @param frame the ReferenceFrame this Coordinate will belong to
   * @param args arguments to pass through to base type
   **/
  template<typename... Args>
  Framed(ReferenceFrame frame, Args&&... args);
#else
  template<typename... Args,
    typename std::enable_if<(sizeof...(Args) >= 1), void*>::type = nullptr>
  Framed(ReferenceFrame frame, Args&&... args)
    : Base(try_transform_to(std::forward<Args>(args), frame)...),
           frame_(frame) {}
#endif

  /**
   * Getter for the ReferenceFrame this Coordinate belongs to.
   *
   * @return the frame
   **/
  const ReferenceFrame &frame() const { return frame_; }

  /**
   * Setter for the ReferenceFrame this Coordinate belongs to. Any further
   * calculations using this Coordinate will use this frame.
   *
   * Not thread-safe.
   *
   * @param new_frame the frame the Coordinate will now belong to
   * @return the old frame
   **/
  ReferenceFrame frame(ReferenceFrame new_frame)
  {
    using std::swap;
    swap(frame_, new_frame);
    return new_frame;
  }

  /**
   * Tests if this Coordinate is within epsilon in distance (as defined by
   * this Coordinate's reference frame's distance metric). If the other
   * Coordinate is in a different reference frame, it is first copied, and
   * converted to this Coordinate's reference frame.
   *
   * @param other the other Coordinate to test against
   * @param epsilon the maximum distance permitted to return true
   * @return true if the distance is less than or equal to  epsilon
   **/
  template<typename Base2>
  bool approximately_equal(const Framed<Base2> &other,
      double epsilon) const
  {
    return std::fabs(self().distance_to(other.self())) < epsilon;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * The four methods below are defined in ReferenceFrame.inl,
   * due to circular dependencies
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /**
   * Copy and transform this coordinate to a new reference frame
   *
   * Requres "ReferenceFrame.h"
   *
   * @param new_frame the frame to transform to
   * @return the new coordinate in the new frame
   *
   * @throws unrelated_frames thrown if the new reference frame is not
   *      part of the same tree as the current one.
   * @throws undefined_transform thrown if no conversion between two frames
   *      along the conversion path has been defined.
   **/
  derived_type WARN_UNUSED
  transform_to(const ReferenceFrame &new_frame) const;

  /**
   * Transform this coordinate, in place, to a new reference frame
   *
   * Requres "ReferenceFrame.h"
   *
   * @param new_frame the frame to transform to
   *
   * @throws unrelated_frames thrown if the new reference frame is not
   *      part of the same tree as the current one.
   * @throws undefined_transform thrown if no conversion between two frames
   *      along the conversion path has been defined.
   **/
  void transform_this_to(const ReferenceFrame &new_frame);

  /**
   * Calculate distance from this Coordinate to a target. If the target
   * is in another reference frame, this and the target will be copied, and
   * converted to their closest common frame.
   *
   * Requres "ReferenceFrame.h"
   *
   * @param target the target Coordinate to calculate distance to
   * @return the distance according to the distance metric in the common
   *   frame, for CoordType. Typically, return will be meters or degrees.
   *
   * @throws unrelated_frames thrown if the target's reference frame is not
   *      part of the same tree as the current one.
   * @throws undefined_transform thrown if no conversion between two frames
   *      along the conversion path has been defined.
   **/
  double distance_to(const derived_type &target) const;

  //using Base::distance_to;

  /**
   * Reduces this Coordinate to it's normalized form, should one exist.
   * Typically useful for Coordinate types which incorporate angles.
   *
   * Requres "ReferenceFrame.h"
   **/
  void normalize();

private:
  ReferenceFrame frame_ = default_frame();
};

} }

#endif
