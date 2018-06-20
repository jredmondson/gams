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
 * @file Stamped.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Contains the Stamped template, which adds a timestamp to coordinates
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_STAMPED_H_
#define _GAMS_POSE_STAMPED_H_

#include "gams/GamsExport.h"
#include <string>
#include <cfloat>
#include <utility>
#include "gams/CPP11_compat.h"
#include "madara/knowledge/containers/DoubleVector.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "ReferenceFrameFwd.h"
#include "Coordinate.h"

namespace gams { namespace pose {

using madara::utility::Clock;
using madara::utility::TimeValue;
using madara::utility::Duration;
using madara::utility::SecondsDuration;

/// Type trait to detect stamped types
MADARA_MAKE_VAL_SUPPORT_TEST(timestamp, x,
    (x.time(), x.time(TimeValue{})));

inline TimeValue try_get_time()
{
  return {};
}

template<typename T, typename... Args>
inline auto try_get_time(const T& v, Args&&...) ->
  typename std::enable_if<supports_timestamp<T>::value, TimeValue>::type
{
  return v.time();
}

template<typename T, typename... Args>
inline auto try_get_time(const T&, Args&&... args) ->
  typename std::enable_if<!supports_timestamp<T>::value, TimeValue>::type
{
  return try_get_time(std::forward<Args>(args)...);
}

/// Internal class implementing coordinates stamped with timestamp.
/// Do not use directly.
template<typename Base>
class Stamped : public Base
{
public:
  using Base::self;

  using derived_type = typename Base::derived_type;

  /**
   * Default Constructor. Initializes timestamp to zero
   **/
  Stamped() = default;

#ifdef DOXYGEN
  /**
   * Construct without a TimeStamp object.
   * Initializes time as zero, unless at least one argument is a type
   * which includes a timestamp, in which case that time will become the stamp
   * of this pose. If multiple do, the first such argument will be used.
   *
   * This overload only participates if the first argument isn't TimeValue
   *
   * @param args arguments to pass through to base type
   **/
  template<typename... Args>
  Stamped(Args&&... args);
#else
  template<typename Arg, typename... Args,
    typename std::enable_if<!std::is_same<typename std::decay<Arg>::type,
      TimeValue>::value && sizeof...(Args) >= 1, void*>::type = nullptr>
  Stamped(Arg&& arg, Args&&... args)
    : Base((Arg &)arg, ((Args &)args)...),
      time_(try_get_time((Arg &)arg, ((Args &)args)...)) {}

  template<typename Arg,
    typename std::enable_if<!std::is_same<typename std::decay<Arg>::type,
      TimeValue>::value, void*>::type = nullptr>
  explicit Stamped(Arg&& arg)
    : Base((Arg &)arg), time_(try_get_time((Arg &)arg)) {}
#endif

  /**
   * Construct using a TimeValue object.
   *
   * @param time the time to stamp with
   **/
  explicit Stamped(TimeValue time)
    : Base(), time_(time) {}

#ifdef DOXYGEN
  /**
   * Construct with a timestamp.
   *
   * @param time the timestamp to use
   * @param args arguments to pass through to base constructor
   **/
  template<typename... Args>
  Stamped(TimeValue time, Args&&... args);
#else
  template<typename... Args,
    typename std::enable_if<(sizeof...(Args) >= 1), void*>::type = nullptr>
  Stamped(TimeValue time, Args&&... args)
    : Base(std::forward<Args>(args)...), time_(time) {}
#endif

  /// Get time as std::chrono based TimeValue
  TimeValue time() const { return time_; }

  /// Set time from std::chrono based TimeValue
  /// @return the new time
  TimeValue time(TimeValue v) { return (time_ = v); }

  /// Get time as nanos since std::steady_clock epoch
  uint64_t nanos() const {
    namespace sc = std::chrono;
    return sc::duration_cast<Duration>(
        time_.time_since_epoch()).count();
  }

  /// Set time as nanos since std::steady_clock epoch
  uint64_t nanos(uint64_t v) {
    Duration dur(v);
    time_ = TimeValue(dur);
    return v;
  }

  /// Get time as seconds since std::steady_clock epoch
  double secs() const {
    namespace sc = std::chrono;
    return sc::duration_cast<SecondsDuration>(
        time_.time_since_epoch()).count();
  }

  /// Set time as seconds since std::steady_clock epoch
  double secs(double v) {
    namespace sc = std::chrono;
    SecondsDuration dur(v);
    Duration ndur(sc::duration_cast<Duration>(dur));
    time_ = TimeValue(ndur);
    return v;
  }

private:
  TimeValue time_;
};

template<typename Base>
inline std::ostream &operator<<(std::ostream &o, const Stamped<Base> &v)
{
  return o << v << "@" << v.time();
}

} }

#endif
