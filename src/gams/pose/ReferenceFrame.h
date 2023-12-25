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
 * @file ReferenceFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_POSE_REFERENCE_FRAME_H_
#define _GAMS_POSE_REFERENCE_FRAME_H_

#include "gams/GamsExport.h"
#include "gams/CPP11_compat.h"
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <utility>
#include <memory>
#include <stdexcept>
#include <mutex>
#include "ReferenceFrameFwd.h"
#include "CartesianFrame.h"
#include "Pose.h"
#include "Quaternion.h"
#include "madara/knowledge/EvalSettings.h"
#include "gams/exceptions/ReferenceFrameException.h"

namespace gams { namespace pose {

class ReferenceFrameVersion;
class ReferenceFrameIdentity;

class ReferenceFrameArena
{
private:
  std::map<std::string,
      std::weak_ptr<ReferenceFrameIdentity>> idents_;

public:
  std::shared_ptr<ReferenceFrameIdentity> lookup(std::string id);

  std::shared_ptr<ReferenceFrameIdentity> find(std::string id) const;

  std::shared_ptr<ReferenceFrameIdentity> make_guid();

  /**
   * Old versions of frames can remain loaded in memory after they are no
   * longer needed. Call this function to clean them out.
   **/
  void gc();
};

/**
 * For internal use. Use ReferenceFrame or FrameStore.
 *
 * Represents a frame's identity, persisting across timestamped versions,
 * including id and type.
 **/
class GAMS_EXPORT ReferenceFrameIdentity
{
public:
  static const uint64_t ETERNAL = ReferenceFrame::ETERNAL;
  static const uint64_t TEMP = ReferenceFrame::TEMP;

private:
  std::string id_;

  static ReferenceFrameArena arena_;
  //static std::map<std::string,
      //std::weak_ptr<ReferenceFrameIdentity>> idents_;

  static uint64_t default_expiry_;

  static std::recursive_mutex idents_lock_;

  mutable std::map<uint64_t, std::weak_ptr<ReferenceFrameVersion>>
    versions_;

  mutable uint64_t expiry_ = ETERNAL;

  mutable std::recursive_mutex versions_lock_;

public:
  /// Public by necessity. Use lookup instead.
  ReferenceFrameIdentity(std::string id, uint64_t expiry)
    : id_(std::move(id)), expiry_(expiry) {}

  static std::shared_ptr<ReferenceFrameIdentity> lookup(std::string id)
  {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);

    return arena_.lookup(std::move(id));
  }

  static std::shared_ptr<ReferenceFrameIdentity> find(std::string id)
  {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);

    return arena_.find(std::move(id));
  }

  static std::shared_ptr<ReferenceFrameIdentity> make_guid()
  {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);

    return arena_.make_guid();
  }

  void register_version(uint64_t timestamp,
      std::shared_ptr<ReferenceFrameVersion> ver) const
  {
    std::lock_guard<std::recursive_mutex> guard(versions_lock_);

    std::weak_ptr<ReferenceFrameVersion> weak(std::move(ver));
    versions_[timestamp] = ver;
  }

  std::shared_ptr<ReferenceFrameVersion> get_version(uint64_t timestamp) const
  {
    std::lock_guard<std::recursive_mutex> guard(versions_lock_);

    auto find = versions_.find(timestamp);

    if (find == versions_.end()) {
      return nullptr;
    }

    return find->second.lock();
  }

  const std::string &id() const { return id_; }

  /**
   * Set the default expiry value for new frames IDs. Setting this will
   * not change any already created frame IDs.
   *
   * If a frame newer than its expiry is saved, saved frames expire
   * of the same ID older than this duration into the past from the
   * timestamp of the new frame.
   *
   * Expired frames are deleted from the KnowledgeBase.
   *
   * Set to ETERNAL (the default) to never expire frames.
   *
   * Note: if a timestamp ETERNAL frame is saved and this is not ETERNAL, all
   * other frames will expire immediately.
   *
   * @return previous default expiry
   **/
  static uint64_t default_expiry(uint64_t age) {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);

    uint64_t ret = default_expiry_;
    default_expiry_ = age;
    return ret;
  }

  /// Return the default expiry for new frame IDs
  static uint64_t default_expiry() {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);
    return default_expiry_;
  }

  /**
   * If a frame newer than this time is saved, expire saved frames
   * of the same ID older than this duration into the past from the
   * timestamp of the new frame.
   *
   * Expired frames are deleted from the KnowledgeBase.
   *
   * Set to ETERNAL (the default) to never expire frames.
   *
   * Note: if a timestamp ETERNAL frame is saved and this is not ETERNAL, all
   * other frames will expire immediately.
   *
   * @return previous expiry
   **/
  uint64_t expiry(uint64_t age) const {
    std::lock_guard<std::recursive_mutex> guard(versions_lock_);

    uint64_t ret = expiry_;
    expiry_ = age;
    return ret;
  }

  /// Return the current expiry
  uint64_t expiry() const {
    std::lock_guard<std::recursive_mutex> guard(versions_lock_);
    return expiry_;
  }

  void expire_older_than(madara::knowledge::KnowledgeBase &kb,
      uint64_t time, const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const;

  static const std::string &default_prefix() {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);
    return FrameEvalSettings::default_prefix();
  }

  /**
   * Old versions of frames can remain loaded in memory after they are no
   * longer needed. Call this function to clean them out.
   **/
  static void gc()
  {
    std::lock_guard<std::recursive_mutex> guard(idents_lock_);

    return arena_.gc();
  }

  void gc_versions();
};

/// Private implementation details
namespace impl {
  inline static std::string &make_kb_key(
      std::string &prefix,
      const std::string &id)
  {
    prefix.reserve(prefix.capacity() + 1 + id.size());

    prefix += ".";
    prefix += id;

    return prefix;
  }

  inline static std::string &make_kb_key(
      std::string &prefix, uint64_t timestamp)
  {
    if (timestamp == ReferenceFrameIdentity::ETERNAL) {
      prefix += ".inf";
    } else {
      prefix += ".";

      std::ostringstream oss;
      oss.fill('0');
      oss.width(16);
      oss << std::hex << timestamp;

      prefix += oss.str();
    }

    return prefix;
  }

  inline static std::string &make_kb_key(
      std::string &prefix,
      const std::string &id, uint64_t timestamp)
  {
    make_kb_key(prefix, id);
    make_kb_key(prefix, timestamp);
    return prefix;
  }
}

/// Type trait to detect stamped types
MADARA_MAKE_VAL_SUPPORT_TEST(nano_timestamp, x,
    (x.nanos(), x.nanos(0UL)));

/// Base case: return default
inline uint64_t try_get_nano_time(uint64_t def)
{
  return def;
}

/// If T supports nanos(), call it and return value
template<typename T, typename... Args>
inline auto try_get_nano_time(uint64_t, const T& v, Args&&...) ->
  typename std::enable_if<supports_nano_timestamp<T>::value, uint64_t>::type
{
  return v.nanos();
}

/// If T doesn't support nanos(), recurse to remaining args
template<typename T, typename... Args>
inline auto try_get_nano_time(uint64_t def, const T&, Args&&... args) ->
  typename std::enable_if<!supports_nano_timestamp<T>::value, uint64_t>::type
{
  return try_get_nano_time(def, std::forward<Args>(args)...);
}

/**
 * For internal use. Use ReferenceFrame or FrameStore instead.
 *
 * Represents a specific frame version; internal implementation of
 * ReferenceFrame, behind a shared_ptr.
 **/
class GAMS_EXPORT ReferenceFrameVersion :
  public std::enable_shared_from_this<ReferenceFrameVersion>
{
private:
  mutable std::shared_ptr<ReferenceFrameIdentity> ident_;
  const ReferenceFrameType *type_;
  uint64_t timestamp_ = ETERNAL;
  Pose origin_;
  mutable bool interpolated_ = false;
  bool temp_ = false;

private:
  template<typename T>
  static uint64_t init_timestamp(uint64_t given, const T &p)
  {
    if (given == ETERNAL) {
      return try_get_nano_time(ETERNAL, p);
    } else {
      return given;
    }
  }

public:
  static const uint64_t ETERNAL = ReferenceFrameIdentity::ETERNAL;
  static const uint64_t TEMP = ReferenceFrameIdentity::TEMP;

  /**
   * Constructor from an origin, and optional timestamp. Will be
   * constructed with Cartesian type, and a random id.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  explicit ReferenceFrameVersion(
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ReferenceFrameVersion({}, Cartesian,
        std::forward<P>(origin),
        timestamp, temp) {}

  /**
   * Constructor from a type, an origin, and optional timestamp. Will be
   * constructed with a random id.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param type a pointer to a ReferenceFrameType struct; typically,
   *        either Cartesian, or GPS.
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  ReferenceFrameVersion(
      const ReferenceFrameType *type,
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ReferenceFrameVersion({}, type,
        std::forward<P>(origin), timestamp, temp) {}

  /**
   * Constructor from a id, an origin, and optional timestamp. Will be
   * constructed with Cartesian type.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param id a string identifier for this frame.
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  ReferenceFrameVersion(
      std::string name,
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ReferenceFrameVersion(
        ReferenceFrameIdentity::lookup(std::move(name)), Cartesian,
        std::forward<P>(origin), timestamp, temp) {}

  /**
   * Constructor from a type, id, an origin, and optional timestamp.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param type a pointer to a ReferenceFrameType struct; typically,
   *        either Cartesian, or GPS.
   * @param id a string identifier for this frame.
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  ReferenceFrameVersion(
      const ReferenceFrameType *type,
      std::string name,
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ReferenceFrameVersion(
        ReferenceFrameIdentity::lookup(std::move(name)), type,
        std::forward<P>(origin), timestamp, temp) {}

  /**
   * Constructor from an existing ReferenceFrameIdentity, an origin,
   * and optional timestamp. Typical users should not use this constructor.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param ident shared_ptr to a ReferenceFrameIdentity, which holds
   *        type and id information.
   * @param id a string identifier for this frame.
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  ReferenceFrameVersion(
      std::shared_ptr<ReferenceFrameIdentity> ident,
      const ReferenceFrameType *type,
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ident_(std::move(ident)),
      type_(type),
      timestamp_(init_timestamp(timestamp, origin)),
      origin_(std::forward<P>(origin)),
      temp_(temp) {}

  /**
   * Constructor from an existing ReferenceFrameIdentity, an origin,
   * and optional timestamp. Typical users should not use this constructor.
   *
   * @tparam a Coordinate type convertible to Pose
   * @param ident shared_ptr to a ReferenceFrameIdentity, which holds
   *        type and id information.
   * @param id a string identifier for this frame.
   * @param origin the origin of this frame, relative to another frame.
   * @param timestamp the timestamp of this frame. By default, will be
   *        treated as "always most current".
   **/
  template<typename P,
    typename std::enable_if<
      supports_transform_to<P>::value, void*>::type = nullptr>
  ReferenceFrameVersion(
      std::shared_ptr<ReferenceFrameIdentity> ident,
      P &&origin,
      uint64_t timestamp = ETERNAL,
      bool temp = false)
    : ReferenceFrameVersion(std::move(ident), Cartesian,
        std::forward<P>(origin), timestamp, temp) {}

  /**
   * Get the ReferenceFrameIdentity object associated with this frame,
   * creating one with random ID if none exists.
   **/
  const ReferenceFrameIdentity &ident() const {
    if (!ident_) {
      ident_ = ReferenceFrameIdentity::make_guid();
    }
    return *ident_;
  }

  /**
   * Retrieve the frame type object for this frame. Mostly useful for
   * comparing to the pose::Cartesian or pose::GPS instances to test
   * what kind of frame this is.
   *
   * @return a pointer to this frames ReferenceFrameType
   **/
  const ReferenceFrameType *type() const { return type_; }

  /**
   * Gets the origin of this Frame
   *
   * @return the Pose which is the origin within this frame's parent,
   * or, a Pose within this own frame, with all zeros for coordinates,
   * if this frame has no parent.
   **/
  const Pose &origin() const {
    return origin_;
  }

  /**
   * Gets the origin of this Frame
   *
   * @return the Pose which is the origin within this frame's parent,
   * or, a Pose within this own frame, with all zeros for coordinates,
   * if this frame has no parent.
   **/
  Pose &mut_origin() {
    return origin_;
  }

  /**
   * Creates a new ReferenceFrame with modified origin
   *
   * @param new_origin the new origin
   * @return the new ReferenceFrame with new origin
   **/
  ReferenceFrame pose(Pose new_origin) const {
    return pose(new_origin, timestamp_);
  }

  /**
   * Creates a new ReferenceFrame with modified origin
   *
   * @param new_origin the new origin
   * @return the new ReferenceFrame with new origin
   **/
  ReferenceFrame move(Position new_origin) const {
    return move(new_origin, timestamp_);
  }

  /**
   * Creates a new ReferenceFrame with modified origin
   *
   * @param new_origin the new origin
   * @return the new ReferenceFrame with new origin
   **/
  ReferenceFrame orient(Orientation new_origin) const {
    return orient(new_origin, timestamp_);
  }

  /**
   * Creates a new ReferenceFrame with modified origin and timestamp
   *
   * @param new_origin the new origin
   * @param timestamp the new timestamp
   * @return the new ReferenceFrame with new origin and timestamp
   **/
  ReferenceFrame pose(const Pose &new_origin, uint64_t timestamp) const {
    return ReferenceFrame(std::make_shared<ReferenceFrameVersion>(
          ident_, type(), new_origin, timestamp));
  }

  /**
   * Creates a new ReferenceFrame with modified origin and timestamp
   *
   * @param new_origin the new origin
   * @param timestamp the new timestamp
   * @return the new ReferenceFrame with new origin and timestamp
   **/
  ReferenceFrame move(const Position &new_origin,
                      uint64_t timestamp) const {
    return pose(Pose(new_origin, Orientation(origin_)), timestamp);
  }

  /**
   * Creates a new ReferenceFrame with modified origin and timestamp
   *
   * @param new_origin the new origin
   * @param timestamp the new timestamp
   * @return the new ReferenceFrame with new origin and timestamp
   **/
  ReferenceFrame orient(const Orientation &new_origin,
                        uint64_t timestamp) const {
    return pose(Pose(Position(origin_), new_origin), timestamp);
  }

  /**
   * Gets the parent frame (the one the origin is within). Will be *this
   * if no parent frame.
   **/
  ReferenceFrame origin_frame() const {
    return origin_.frame();
  }

  /**
   * Equality operator.
   *
   * @param other the frame to compare to.
   * @return true if both frames are the same object (i.e., same address).
   *    Otherwise, frames are not considered equal (returns false).
   **/
  bool operator==(const ReferenceFrame &other) const;

  /**
   * Inequality operator.
   *
   * @param other the frame to compare to.
   * @return false if both frames are the same object (i.e., same address).
   *    Otherwise, frames are not considered equal (returns true).
   **/
  bool operator!=(const ReferenceFrame &other) const;

  /**
   * Equality operator.
   *
   * @param other the frame to compare to.
   * @return true if both frames are the same object (i.e., same address).
   *    Otherwise, frames are not considered equal (returns false).
   **/
  bool operator==(const ReferenceFrameVersion &other) const;

  /**
   * Inequality operator.
   *
   * @param other the frame to compare to.
   * @return false if both frames are the same object (i.e., same address).
   *    Otherwise, frames are not considered equal (returns true).
   **/
  bool operator!=(const ReferenceFrameVersion &other) const;

  /**
   * Returns a human-readable name for the reference frame type
   *
   * @return the name reference frame type (e.g., GPS, Cartesian)
   **/
  const char *name() const {
    return type()->name;
  }

  /**
   * Get the ID string of this frame. Generates a random ID for this frame
   * if it doesn't already have one. This ID will not change until this
   * frame is destructed.
   **/
  const std::string &id() const {
    return ident().id();
  }

  /**
   * Does this frame have an ID? Frames gain an ID either at construction,
   * or lazily as needed (by having the id() method, or save* methods
   * called). This method can be useful to prevent, e.g., logging code from
   * incurring the overhead of generating an ID for an ephemeral frame.
   *
   * @return true if this frame already has an ID, false if not
   **/
  bool has_id() const {
    return (bool)ident_;
  }

  /**
   * Get the ID string of this frame. Will not generate an ID if this frame
   * doesn't have one. In that case, returns nullptr.
   *
   * @return nullptr has_id() is false; otherwise, a pointer to the result
   *         of id()
   **/
  const std::string *id_ptr() const {
    return has_id() ? nullptr : &id();
  }

  /**
   * Get the timestamp assigned to this frame.
   **/
  uint64_t timestamp() const {
    return timestamp_;
  }

  /**
   * Clone the this frame, but with new timestamp.
   *
   * @return the new frame object
   **/
  ReferenceFrame timestamp(uint64_t timestamp) const {
    return ReferenceFrame(std::make_shared<ReferenceFrameVersion>(
          ident_, type(), origin(), timestamp));
  }

  /**
   * Returns true if this frame was interpolated from two stored frames.
   * If this frame is itself stored, it will no longer be considered as
   * interpolated.
   **/
  bool interpolated() const {
    return interpolated_;
  }

  /**
   * Returns true if this frame is a temporary; one invented to serve as
   * root of a frame tree.
   **/
  bool temp() const {
    return temp_;
  }

  static const std::string &default_prefix() {
    return ReferenceFrameIdentity::default_prefix();
  }

  /**
   * Returns the key that save() will use to store this frame.
   **/
  std::string key(const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const {
    std::string prefix = settings.prefix();
    impl::make_kb_key(prefix, id(), timestamp());
    return prefix;
  }

  /**
   * Save this ReferenceFrame to the knowledge base,
   * The saved frames will be marked with their timestamp for later
   * retrieval. If timestamp is ETERNAL, it will always be treated as the most
   * recent frame.
   *
   * @param kb the KnowledgeBase to store into
   * @param expiry use this expiry time instead of the one set on this ID
   **/
  void save(madara::knowledge::KnowledgeBase &kb,
            uint64_t expiry,
            const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const {
    std::string key = this->key(settings);
    save_as(kb, std::move(key), expiry, settings);
  }

  /**
   * Save this ReferenceFrame to the knowledge base,
   * The saved frames will be marked with their timestamp for later
   * retrieval. If timestamp is ETERNAL, it will always be treated as the most
   * recent frame.
   *
   * @param kb the KnowledgeBase to store into
   **/
  void save(madara::knowledge::KnowledgeBase &kb,
      const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const {
    std::string key = this->key(settings);
    save_as(kb, std::move(key), settings);
  }

  /**
   * Load a single ReferenceFrame, by ID and timestamp. Will not
   * interpolate. Returns an invalid frame if none exists with given
   * ID and timestamp.
   *
   * @param id the ID of the frame to load
   * @param timestamp of frame to load. ETERNAL is matched exactly; it
   *   will only return a frame if one with that timestamp exists.
   * @param parent_timsteamp timestamp of parent to load. Parent is
   *   loaded using load()
   *
   * @return the loaded ReferenceFrame, or an invalid frame if none
   *         exists.
   **/
  static ReferenceFrame load_exact(
          madara::knowledge::KnowledgeBase &kb,
          const std::string &id,
          uint64_t timestamp = ETERNAL,
          uint64_t parent_timestamp = ETERNAL,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT,
          bool throw_on_errors = true,
          ReferenceFrameArena* arena = nullptr);

private:
  /**
   * Load a single ReferenceFrame, by ID and timestamp. Will not
   * interpolate. Returns an invalid frame if none exists with given
   * ID and timestamp.
   *
   * @param id the ID of the frame to load
   * @param timestamp of frame to load. ETERNAL is matched exactly; it
   *   will only return a frame if one with that timestamp exists.
   * @param parent_timsteamp timestamp of parent to load. Parent is
   *   loaded using load()
   *
   * @return the loaded ReferenceFrame, or an invalid frame if none
   *         exists.
   **/
  static ReferenceFrame load_exact_internal(
          madara::knowledge::KnowledgeBase &kb,
          const std::string &id,
          uint64_t timestamp = ETERNAL,
          uint64_t parent_timestamp = ETERNAL,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT,
          bool throw_on_errors = true,
          ReferenceFrameArena* arena = nullptr);

public:
  /**
   * Load a single ReferenceFrame, by ID and timestamp, interpolated
   * if applicable.
   *
   * @param id the ID of the frame to load
   * @param timestamp if ETERNAL, gets the latest frame (no interpolation)
   *   Otherwise, gets the frame at a specified timestamp,
   *   interpolated necessary.
   *
   * @return the loaded ReferenceFrame, or an invalid frame if none
   *         exists.
   **/
  static ReferenceFrame load(
          madara::knowledge::KnowledgeBase &kb,
          const std::string &id,
          uint64_t timestamp = ETERNAL,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT,
          bool throw_on_errors = true,
          ReferenceFrameArena* arena = nullptr);

  /**
   * Get the latest available timestamp in the knowledge base
   * for the given id.
   *
   * @param kb the knowledge base to search
   * @param id the id to search for
   *
   * @return the latest timestamp for the id in kb. If timestamp ETERNAL is
   *         available, that will be returned.
   **/
  static uint64_t latest_timestamp(
          madara::knowledge::KnowledgeBase &kb,
          const std::string &id,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT);

private:
  using ancestor_elem = std::pair<std::string, uint64_t>;
  using ancestor_vec = std::vector<ancestor_elem>;

  static std::pair<uint64_t, uint64_t> find_nearest_neighbors(
      madara::knowledge::KnowledgeBase &kb, const std::string &id,
      uint64_t timestamp, const FrameEvalSettings &settings);

  static ancestor_vec get_ancestry(
      madara::knowledge::KnowledgeBase &kb,
      std::string name,
      const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT);

  static uint64_t find_common_timestamp_to_first_ancestor(
      const std::vector<ancestor_vec> &stamps);

public:
  /**
   * Get the latest available timestamp in the knowledge base
   * common to all the given ids. Will return ETERNAL if no common
   * timestamp is available.
   *
   * @param kb the knowledge base to search
   * @param begin iterator which dereferences to std::string
   * @param end ending iterator
   * @tparam InputIterator an InputIterator type, such as std::vector::iterator
   *
   * @return the latest timestamp for the id in kb. If timestamp ETERNAL is
   *         available, that will be returned.
   **/
  template<typename InputIterator>
  static uint64_t latest_common_timestamp(
          madara::knowledge::KnowledgeBase &kb,
          InputIterator begin,
          InputIterator end,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT)
  {
    if (begin == end) {
      return 0UL - 1;
    }

    std::vector<ancestor_vec> stamps;

    if (std::is_same<
        typename std::iterator_traits<InputIterator>::iterator_category,
        std::random_access_iterator_tag>::value) {
      size_t count = std::distance(begin, end);

      if (count == 1) {
        madara::knowledge::ContextGuard guard(kb);

        return find_nearest_neighbors(kb, *begin, -1, settings).first;
      }

      stamps.reserve(count);
    }

    {
      madara::knowledge::ContextGuard guard(kb);

      InputIterator cur = begin;
      while (cur != end) {
        stamps.emplace_back(get_ancestry(kb, *cur, settings));
        ++cur;
      }
    }

    return find_common_timestamp_to_first_ancestor(stamps);
  }

  /**
   * Get the latest available timestamp in the knowledge base
   * common to all the given ids. Will return ETERNAL if no common
   * timestamp is available.
   *
   * @param kb the knowledge base to search
   * @param container container of std::string
   * @tparam Container a container type (such as std::vector)
   *
   * @return the latest timestamp for the id in kb. If timestamp ETERNAL is
   *         available, that will be returned.
   **/
  template<typename Container>
  static uint64_t latest_common_timestamp(
          madara::knowledge::KnowledgeBase &kb,
          const Container &ids,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT)
  {
    madara::knowledge::ContextGuard guard(kb);

    return latest_common_timestamp(kb, ids.cbegin(), ids.cend(),
        settings);
  }

  /**
   * Load ReferenceFrames, by ID, and their common ancestors. Will
   * interpolate frames to ensure the returned frames all have a common
   * timestamp.
   *
   * @tparam a ForwardIterator, of item type std::string
   *
   * @param begin beginning iterator
   * @param end ending iterator
   * @param timestamp if ETERNAL, the latest possible tree will be returned.
   *   Otherwise, the specified timestamp will be returned.
   *
   * @return a vector of ReferenceFrames, each corresponding to the
   *   input IDs, in the same order. If the timestamp specified cannot
   *   be satisfied, returns an empty vector.
   **/
  template<typename ForwardIterator>
  static std::vector<ReferenceFrame> load_tree(
          madara::knowledge::KnowledgeBase &kb,
          ForwardIterator begin,
          ForwardIterator end,
          uint64_t timestamp = ETERNAL,
          const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT,
          ReferenceFrameArena* arena = nullptr)
  {
    std::vector<ReferenceFrame> ret;
    if (std::is_same<
        typename std::iterator_traits<ForwardIterator>::iterator_category,
        std::random_access_iterator_tag>::value) {
      size_t count = std::distance(begin, end);
      ret.reserve(count);
    }

    madara::knowledge::ContextGuard guard(kb);

    if (timestamp == ETERNAL) {
      timestamp = latest_common_timestamp(kb, begin, end, settings);
    }

    ReferenceFrameArena local_arena;
    if (arena == nullptr) {
      arena = &local_arena;
    }

    while (begin != end) {
      ReferenceFrame frame = load(kb, *begin, timestamp, settings, arena);
      if (!frame.valid()) {
        std::stringstream msg;
        msg << "ReferenceFrame::load_tree: could not find frame \"" <<
            *begin << "\" at timestamp " << timestamp << std::endl;
        throw exceptions::ReferenceFrameException(msg.str());
      }
      ret.push_back(frame);
      ++begin;
    }
    throw_if_not_connected(ret);
    return ret;
  }

  static bool check_is_connected(const std::vector<ReferenceFrame> &frames);
  static void throw_if_not_connected(const std::vector<ReferenceFrame> &frames);

  /**
   * Load ReferenceFrames, by ID, and their common ancestors. Will
   * interpolate frames to ensure the returned frames all have a common
   * timestamp.
   *
   * @tparam a Container, supporting cbegin() and cend(),
   *    of item type std::string
   *
   * @param ids a Container of ids
   * @param timestamp if ETERNAL, the latest possible tree will be returned.
   *   Otherwise, the specified timestamp will be returned.
   *
   * @return a vector of ReferenceFrames, each corresponding to the
   *   input IDs, in the same order. If the timestamp specified cannot
   *   be satisfied, returns an empty vector.
   **/
  template<typename Container>
  static std::vector<ReferenceFrame> load_tree(
      madara::knowledge::KnowledgeBase &kb, const Container &ids,
      uint64_t timestamp = ETERNAL,
      const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT,
      ReferenceFrameArena* arena = nullptr)
  {
    return load_tree(kb, ids.begin(), ids.end(),
        timestamp, std::move(settings), arena);
  }

  /**
   * Save this ReferenceFrame to the knowledge base,
   * with a specific key value.
   *
   * @param kb the KnowledgeBase to save to
   * @param key a key prefix to save with
   * @param expiry use this expiry time instead of the one set on this ID
   **/
  void save_as(madara::knowledge::KnowledgeBase &kb,
      std::string key, uint64_t expiry,
      const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const;
  /**
   * Save this ReferenceFrame to the knowledge base,
   * with a specific key value.
   *
   * @param kb the KnowledgeBase to save to
   * @param key a key prefix to save with
   **/
  void save_as(madara::knowledge::KnowledgeBase &kb,
      std::string key,
      const FrameEvalSettings &settings = FrameEvalSettings::DEFAULT) const
  {
    save_as(kb, key, ident().expiry(), settings);
  }

  /**
   * Interpolate a frame between the given frame; use the given parent.
   * Note: no sanity checking is done. Ensure that parent has a compatible
   * timestamp, and that this and other are the same frame at different
   * times.
   *
   * @param other the other frame to interpolate towards.
   * @param parent the parent the returned frame will have.
   * @param time the timestamp to interpolate at.
   * @return the interpolated frame.
   **/
  ReferenceFrame interpolate(const ReferenceFrame &other, ReferenceFrame parent, uint64_t time) const
  {
    //std::cerr << "Interpolate has_id: " << has_id() << std::endl;
    if (has_id()) {
      auto ret = ident().get_version(time);
      if (ret) {
        return ret;
      }
    }
    double fraction = (time - timestamp()) / (double)(other.timestamp() - timestamp());

    Pose interp = origin();
    const Pose &opose = other.origin();

    interp.frame(std::move(parent));

    interp.x((fraction * (opose.x() - interp.x())) + interp.x());
    interp.y((fraction * (opose.y() - interp.y())) + interp.y());
    interp.z((fraction * (opose.z() - interp.z())) + interp.z());

    Quaternion iq(origin().as_orientation_vec());
    Quaternion oq(opose.as_orientation_vec());

    iq.slerp_this(oq, fraction);
    iq.to_angular_vector(interp.as_orientation_vec());

    //std::cerr << "Interp " << fraction << " " << origin() << "  " << interp << "  " << opose << std::endl;

    auto ret = std::make_shared<ReferenceFrameVersion>(
        ident_, type(), std::move(interp), time);

    if (has_id()) {
      ident().register_version(time, ret);
    }

    ret->interpolated_ = true;
    return ret;
  }

  template<typename CoordType>
  friend class Coordinate;

private:
  bool check_consistent() const;
};

/**
 * Class for storing and loading frames in a given KnowledgeBase, using
 * given settings, with a set expiry.
 *
 * This class itself is immutable, and thus trivially thread-safe.
 **/
class GAMS_EXPORT FrameStore {
private:
  /*
   * KnowledgeBase uses const to indicate not changing the underlying
   * KnowledgeBaseImpl, as well as itself. We won't be changing the
   * KnowledgeBase object itself, and will only call thread-safe methods.
   */
  mutable madara::knowledge::KnowledgeBase kb_;

  FrameEvalSettings settings_ = FrameEvalSettings::DEFAULT;
  uint64_t expiry_;

public:
  static const uint64_t ETERNAL = ReferenceFrameIdentity::ETERNAL;
  static const uint64_t TEMP = ReferenceFrameIdentity::TEMP;

  /**
   * Primary constructor for FrameStore
   *
   * @param kb the KnowledgeBase to use
   * @param prefix within the KnowledgeBase to use
   * @param expiry expiration to use for all saves
   **/
  FrameStore(madara::knowledge::KnowledgeBase kb,
             FrameEvalSettings settings, uint64_t expiry)
    : kb_(std::move(kb)), settings_(std::move(settings)), expiry_(expiry) {}

  /**
   * Constructor for FrameStore
   * Uses ReferenceFrame::default_expiry() for expiration
   *
   * @param kb the KnowledgeBase to use
   * @param settings to use
   **/
  FrameStore(madara::knowledge::KnowledgeBase kb, FrameEvalSettings settings)
    : FrameStore(std::move(kb), std::move(settings),
        ReferenceFrame::default_expiry()) {}

  /**
   * Constructor for FrameStore
   * Uses default FrameEvalSettings
   *
   * @param kb the KnowledgeBase to use
   * @param expiry expiration to use for all saves
   **/
  FrameStore(madara::knowledge::KnowledgeBase kb, uint64_t expiry)
    : FrameStore(std::move(kb), FrameEvalSettings::DEFAULT, expiry) {}


  /**
   * Constructor for FrameStore
   * Uses default FrameEvalSettings
   * Uses ReferenceFrame::default_expiry() for expiration
   *
   * @param kb the KnowledgeBase to use
   **/
  FrameStore(madara::knowledge::KnowledgeBase kb)
    : FrameStore(std::move(kb), FrameEvalSettings::DEFAULT,
        ReferenceFrame::default_expiry()) {}

  /// Return the current expiry for all frames saved with this FrameStore
  /// See ReferenceFrame::expiry(uint64_t) for details of expiration.
  uint64_t expiry() const { return expiry_; }

  /// Return the current settings for frames saved/loaded with this FrameStore
  const FrameEvalSettings &settings() const { return settings_; }

  /// Return the KnowledgeBase to load/save with this FrameStore
  const madara::knowledge::KnowledgeBase &kb() const { return kb_; }

  /**
   * Save a ReferenceFrame to the knowledge base,
   * The saved frames will be marked with their timestamp for later
   * retrieval. If timestamp is ETERNAL, it will always be treated as the most
   * recent frame.
   *
   * @param frame the ReferenceFrame to store
   **/
  void save(const ReferenceFrame &frame) const {
    return frame.save(kb_, expiry_, settings_);
  }

  /**
   * Load a single ReferenceFrame, by ID.
   *
   * @param id the ID of the frame to load
   * @param timestamp if ETERNAL, gets the latest frame (no interpolation)
   *   Otherwise, gets the frame at a specified timestamp,
   *   interpolated necessary.
   *
   * @return the imported ReferenceFrame, or an invalid frame if none
   *         exists.
   **/
   ReferenceFrame load(const std::string &id, uint64_t timestamp = ETERNAL) {
     return ReferenceFrame::load(kb_, id, timestamp, settings_);
   }

  /**
   * Load ReferenceFrames, by ID, and their common ancestors. Will
   * interpolate frames to ensure the returned frames all have a common
   * timestamp.
   *
   * @tparam an InputIterator, of item type std::string
   *
   * @param begin beginning iterator
   * @param end ending iterator
   * @param timestamp if ETERNAL, the latest possible tree will be returned.
   *   Otherwise, the specified timestamp will be returned.
   *
   * @return a vector of ReferenceFrames, each corresponding to the
   *   input IDs, in the same order. If the timestamp specified cannot
   *   be satisfied, returns an empty vector.
   **/
  template<typename InputIterator>
  std::vector<ReferenceFrame> load_tree(
        InputIterator begin,
        InputIterator end,
        uint64_t timestamp = ETERNAL) const {
   return ReferenceFrame::load_tree(kb_, begin, end, timestamp, settings_);
  }

  /**
   * Load ReferenceFrames, by ID, and their common ancestors. Will
   * interpolate frames to ensure the returned frames all have a common
   * timestamp.
   *
   * @tparam a Container, supporting cbegin() and cend(),
   *    of item type std::string
   *
   * @param ids a Container of ids
   * @param timestamp if ETERNAL, the latest possible tree will be returned.
   *   Otherwise, the specified timestamp will be returned.
   *
   * @return a vector of ReferenceFrames, each corresponding to the
   *   input IDs, in the same order. If the timestamp specified cannot
   *   be satisfied, returns an empty vector.
   **/
  template<typename Container>
  std::vector<ReferenceFrame> load_tree(
        const Container &ids,
        uint64_t timestamp = ETERNAL) const {
   return ReferenceFrame::load_tree(kb_, ids, timestamp, settings_);
  }
};

/**
 * Helper function to find the common frame between two frames.
 *
 * @param from the initial frame
 * @param to the target frame
 * @param to_stack if not nullptr, the frames needed to go from base to
 *  target frame will be pushed to pointed to vector
 **/
GAMS_EXPORT const ReferenceFrame *find_common_frame(
    const ReferenceFrame *from,
    const ReferenceFrame *to,
    std::vector<const ReferenceFrame *> *to_stack = nullptr);

/**
 * Thrown when an an attempt is made to transform between frames
 * that do not belong to the same frame tree.
 **/
class unrelated_frames : public std::runtime_error
{
public:
  /**
   * The only Constructor.
   *
   * @param from_frame the frame the coordinate belongs to
   * @param to_frame the frame the coordinate is being transformed to
   **/
  unrelated_frames(ReferenceFrame from_frame, ReferenceFrame to_frame);
  ~unrelated_frames() throw();

  ReferenceFrame from_frame;
  ReferenceFrame to_frame;
};

/**
 * Thrown when an attempt is made to transform between two frame types,
 * and there's not transform defined.
 *
 * Note that between two frame types A and B, there are four different
 * kinds of transforms, which must be defined individually:
 *    1) From A as parent to B as its child
 *    2) From A as child to B as its parent
 *    3) From B as parent to A as its child
 *    4) From B as child to A as its parent
 **/
class undefined_transform : public std::runtime_error
{
public:
  /**
   * The only Constructor
   *
   * @param parent_frame of the two involved frames, the parent frame
   * @param child_frame of the two involved frames, the child frame
   * @param is_child_to_parent indicates direction of transformation
   * @param unsupported_angular true if the error was due to orientd
   *   reference frames not being supported for this transformation.
   *   Defaults to false;
   **/
  undefined_transform(
    const ReferenceFrameType *parent_frame,
    const ReferenceFrameType *child_frame,
    bool is_child_to_parent,
    bool unsupported_angular = false);
  ~undefined_transform() throw();

  const ReferenceFrameType *parent_frame;
  const ReferenceFrameType *child_frame;
  bool is_child_to_parent;
  bool unsupported_angular;
};

/**
 * For internal use.
 *
 * Provides implementation of angular and pose transforms for frames
 * where angular transformation is independant of linear. This applies
 * to, for example, Cartesian and GPS frames, but not UTM frames.
 * Inherit from this to use this implementation.
 **/
namespace simple_rotate {
  /**
   * Rotates a LinearVector according to a AngularVector
   *
   * @param x   the x coordinate to orient (in-place)
   * @param y   the y coordinate to orient (in-place)
   * @param z   the z coordinate to orient (in-place)
   * @param rx  the angular x to apply, axis-angle notation
   * @param ry  the angular y to apply, axis-angle notation
   * @param rz  the angular z to apply, axis-angle notation
   * @param reverse if true, apply angular in opposite direction
   **/
  void orient_linear_vec(
      double &x, double &y, double &z,
      double rx, double ry, double rz,
      bool reverse = false);

  /**
   * Transform AngularVector in-place into its origin frame from this frame
   *
   * @param origin the origin frame
   * @param self the current reference frame
   * @param orx  the x component of the origin axis-angle representation
   * @param ory  the y component of the origin axis-angle representation
   * @param orz  the z component of the origin axis-angle representation
   * @param rx  the x component of the axis-angle representation
   * @param ry  the y component of the axis-angle representation
   * @param rz  the z component of the axis-angle representation
   **/
  void transform_angular_to_origin(
                  const ReferenceFrameType *origin,
                  const ReferenceFrameType *self,
                  double orx, double ory, double orz,
                  double &rx, double &ry, double &rz);

  /**
   * Transform AngularVector in-place from its origin frame
   *
   * @param origin the origin frame
   * @param self the current reference frame
   * @param orx  the x component of the origin axis-angle representation
   * @param ory  the y component of the origin axis-angle representation
   * @param orz  the z component of the origin axis-angle representation
   * @param rx  the x component of the axis-angle representation
   * @param ry  the y component of the axis-angle representation
   * @param rz  the z component of the axis-angle representation
   **/
  void transform_angular_from_origin(
                  const ReferenceFrameType *origin,
                  const ReferenceFrameType *self,
                  double orx, double ory, double orz,
                  double &rx, double &ry, double &rz);

  /**
   * Transform pose in-place into its origin frame from this frame.
   * Simply applies linear and angular transforms independantly
   *
   * @param origin the origin frame
   * @param self the current reference frame
   * @param ox the x axis for the coordinate to translate
   * @param oy the y axis for the coordinate to translate
   * @param oz the z axis for the coordinate to translate
   * @param orx  the x component of the origin axis-angle representation
   * @param ory  the y component of the origin axis-angle representation
   * @param orz  the z component of the origin axis-angle representation
   * @param rx  the x component of the axis-angle representation
   * @param ry  the y component of the axis-angle representation
   * @param rz  the z component of the axis-angle representation
   **/
  void transform_pose_to_origin(
                  const ReferenceFrameType *origin,
                  const ReferenceFrameType *self,
                  double ox, double oy, double oz,
                  double orx, double ory, double orz,
                  double &x, double &y, double &z,
                  double &rx, double &ry, double &rz,
                  bool fixed);

  /**
   * Transform pose in-place from its origin frame
   * Simply applies linear and angular transforms independantly
   *
   * @param origin the origin frame
   * @param self the current reference frame
   * @param x the x axis for the coordinate to translate
   * @param y the y axis for the coordinate to translate
   * @param z the z axis for the coordinate to translate
   * @param ox the x axis for the coordinate to translate
   * @param oy the y axis for the coordinate to translate
   * @param oz the z axis for the coordinate to translate
   * @param orx  the x component of the origin axis-angle representation
   * @param ory  the y component of the origin axis-angle representation
   * @param orz  the z component of the origin axis-angle representation
   * @param rx  the x component of the axis-angle representation
   * @param ry  the y component of the axis-angle representation
   * @param rz  the z component of the axis-angle representation
   **/
  void transform_pose_from_origin(
                  const ReferenceFrameType *origin,
                  const ReferenceFrameType *self,
                  double ox, double oy, double oz,
                  double orx, double ory, double orz,
                  double &x, double &y, double &z,
                  double &rx, double &ry, double &rz,
                  bool fixed);

  /**
   * Calculates smallest angle between two AngularVectors
   *
   * @param self the current reference frame
   * @param rx1  the starting angular on x axis
   * @param ry1  the starting angular on y axis
   * @param rz1  the starting angular on z axis
   * @param rx2 the ending rotatation on x axis
   * @param ry2  the ending angular on y axis
   * @param rz2  the ending angular on z axis
   * @return the difference in radians
   **/
  double calc_angle(
                  const ReferenceFrameType *self,
                  double rx1, double ry1, double rz1,
                  double rx2, double ry2, double rz2);
}

/**
 * Returns a default normalized positional
 **/
inline void default_normalize_linear(
            const ReferenceFrameType *,
            double &, double &, double &) {}

/**
 * Returns a default normalized angular
 **/
inline void default_normalize_angular(
            const ReferenceFrameType *,
            double &, double &, double &) {}

  /**
   * Returns a default normalized pose
   *
   * @param self the current reference frame
   * @param x    the x pose coordinate
   * @param y    the y pose coordinate
   * @param z    the z pose coordinate
   * @param rx   the ending angular on x axis
   * @param ry   the ending angular on y axis
   * @param rz   the ending angular on z axis
   * @return the difference in radians
   **/
inline void default_normalize_pose(
                const ReferenceFrameType *self,
                double &x, double &y, double &z,
                double &rx, double &ry, double &rz)
{
  self->normalize_linear(self, x, y, z);
  self->normalize_angular(self, rx, ry, rz);
}

} }

#include "ReferenceFrame.inl"

#endif
