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
 * This file contains forward declarations for ReferenceFrame objects
 **/

#ifndef _GAMS_POSE_REFERENCE_FRAME_FWD_H_
#define _GAMS_POSE_REFERENCE_FRAME_FWD_H_

#include <gams/GamsExport.h>
#include <gams/CPP11_compat.h>
#include <madara/knowledge/KnowledgeBase.h>
#include <stdexcept>
#include <sstream>

namespace gams
{
  namespace pose
  {
    class ReferenceFrameIdentity;
    class ReferenceFrameVersion;
    struct ReferenceFrameType;
    class Pose;
    class Position;
    class Orientation;

    /**
     * Provides Reference Frame (i.e., coordinate systemm) transforms.
     *
     * A ReferenceFrame has:
     *
     * * a type: either GPS or Cartesian (the default). Others may be
     *    implemented in the future.
     *
     * * an ID: some string that should be unique within each knowledge base.
     *    By default, a random GUID will be generated if none is given when
     *    constructing a ReferenceFrame. The ID should be unique per platform,
     *    across all KnowledgeBases that platform might use.
     *    The same ID saved to different KnowledgeBase objects will be assumed
     *    to represent the same frame, interchangeably.
     *
     * * a timestamp: a timestamp representing at what time the transform was
     *    measured or derived. No units are assumed by GAMS, but interpolation
     *    assumes that timestamps progress linearly with respect to real time,
     *    and monotonically. A -1, the default if none is given, is treated
     *    as "always correct" at all times.
     *
     * * an origin: a Pose in another frame which is the location and
     *    of this frame's origin with respect to that frame.
     *
     * ReferenceFrame objects are immutable. "Setters" like timestamp() and
     * pose() return a new ReferenceFrame object modified accordingly.
     *
     * If you use gps_frame() or default_frame(), and wish to save any frames
     * which involve them as ancestors, ensure you save them to any
     * KnowledgeBase you will be saving such frames to.
     *
     * ReferenceFrames get saved to KnowledgeBases under the ".gams.frames"
     * prefix. Do not modify keys under this prefix directly.
     *
     * ReferenceFrame objects are ref-counted proxies for an underlying object.
     * As such, they are cheap and safe to pass and return by value.
     **/
    class GAMS_EXPORT ReferenceFrame
    {
    private:
      std::shared_ptr<ReferenceFrameVersion> impl_;

    public:
      /**
       * Default constructor. This frame's valid() will return false. Calling
       * any other method is undefined behavior.
       *
       * If a frame has a Pose as origin with an invalid frame, it will be
       * treated as a parent-less frame.
       **/
      ReferenceFrame() : impl_() {}

      /**
       * Construct from an existing ReferenceFrameVersion object. In general,
       * you should not be constructing ReferenceFrameVersion objects directly.
       * Use the other constructors of this class.
       *
       * @param impl a shared_ptr to construct with. This object will act as
       *        a proxy for the pointed-to object.
       **/
      ReferenceFrame(std::shared_ptr<ReferenceFrameVersion> impl) :
        impl_(std::move(impl)) {}

      /**
       * Constructor from an origin, and optional timestamp. Will be
       * constructed with Cartesian type, and a random id.
       *
       * @tparam a Coordinate type convertible to Pose
       * @param origin the origin of this frame, relative to another frame.
       * @param timestamp the timestamp of this frame. By default, will be
       *        treated as "always most current".
       **/
      template<typename = void>
      explicit ReferenceFrame(
          const Pose &origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              origin, timestamp)) {}

      /**
       * Constructor from an origin, and optional timestamp. Will be
       * constructed with Cartesian type, and a random id.
       *
       * @tparam a Coordinate type convertible to Pose
       * @param origin the origin of this frame, relative to another frame.
       * @param timestamp the timestamp of this frame. By default, will be
       *        treated as "always most current".
       **/
      template<typename = void>
      explicit ReferenceFrame(
          Pose &&origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              std::move(origin), timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          const ReferenceFrameType *type,
          const Pose &origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              type, origin, timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          const ReferenceFrameType *type,
          Pose &&origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              type, std::move(origin), timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          std::string id,
          const Pose &origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              id, origin, timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          std::string id,
          Pose &&origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              id, std::move(origin), timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          const ReferenceFrameType *type,
          std::string id,
          const Pose &origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              type, id, origin, timestamp)) {}

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
      template<typename = void>
      ReferenceFrame(
          const ReferenceFrameType *type,
          std::string id,
          Pose &&origin,
          uint64_t timestamp = -1)
        : impl_(std::make_shared<ReferenceFrameVersion>(
              type, id, std::move(origin), timestamp)) {}

      /**
       * Test whether this frame is valid. If not, all other methods will
       * have undefined behavior.
       *
       * @return true if valid, false otherwise.
       **/
      bool valid() const { return (bool)impl_; }

      /**
       * Gets the origin of this Frame
       *
       * @return the Pose which is the origin within this frame's parent,
       * or, a Pose within this own frame, with all zeros for coordinates,
       * if this frame has no parent.
       **/
      const Pose &origin() const;

      /**
       * Creates a new ReferenceFrame with modified origin
       *
       * @param new_origin the new origin
       * @return the new ReferenceFrame with new origin
       **/
      ReferenceFrame pose(const Pose &new_origin) const;

      /**
       * Creates a new ReferenceFrame with modified origin
       *
       * @param new_origin the new origin
       * @return the new ReferenceFrame with new origin
       **/
      ReferenceFrame move(const Position &new_origin) const;

      /**
       * Creates a new ReferenceFrame with modified origin
       *
       * @param new_origin the new origin
       * @return the new ReferenceFrame with new origin
       **/
      ReferenceFrame orient(const Orientation &new_origin) const;

      /**
       * Creates a new ReferenceFrame with modified origin and timestamp
       *
       * @param new_origin the new origin
       * @param timestamp the new timestamp
       * @return the new ReferenceFrame with new origin and timestamp
       **/
      ReferenceFrame pose(const Pose &new_origin, uint64_t timestamp) const;

      /**
       * Creates a new ReferenceFrame with modified origin and timestamp
       *
       * @param new_origin the new origin
       * @param timestamp the new timestamp
       * @return the new ReferenceFrame with new origin and timestamp
       **/
      ReferenceFrame move(const Position &new_origin, uint64_t timestamp) const;

      /**
       * Creates a new ReferenceFrame with modified origin and timestamp
       *
       * @param new_origin the new origin
       * @param timestamp the new timestamp
       * @return the new ReferenceFrame with new origin and timestamp
       **/
      ReferenceFrame orient(const Orientation &new_origin, uint64_t timestamp) const;

      /**
       * Gets the parent frame (the one the origin is within). Will be *this
       * if no parent frame.
       **/
      ReferenceFrame origin_frame() const;

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
       * Returns a human-readable name for the reference frame type
       *
       * @return the name reference frame type (e.g., GPS, Cartesian)
       **/
      std::string name() const;

      /**
       * Get the ID string of this frame. By default, frames generate a
       * random GUID as their ID
       **/
      const std::string &id() const;

      std::ostringstream key_stream() const;

      std::string key() const;

      /**
       * Retrieve the frame type object for this frame. Mostly useful for
       * comparing to the pose::Cartesian or pose::GPS instances to test
       * what kind of frame this is.
       *
       * @return a pointer to this frames ReferenceFrameType
       **/
      const ReferenceFrameType *type() const;

      /**
       * Get the timestamp assigned to this frame.
       *
       * @return the timestamp
       **/
      uint64_t timestamp() const;

      /**
       * Clone the this frame, but with new timestamp.
       *
       * @return the new frame object
       **/
      ReferenceFrame timestamp(uint64_t) const;

      /**
       * Sets configuration for all frames of this frames ID.
       *
       * If a frame newer than this time is saved, expire saved frames
       * of the same ID older than this duration into the past from the
       * timestamp of the new frame.
       *
       * Expired frames are deleted from the KnowledgeBase.
       *
       * Set to -1 (the default) to never expire frames.
       *
       * Note: if a timestamp -1 frame is saved and this is not -1, all
       * other frames will expire immediately.
       *
       * @return previous expiry
       **/
      uint64_t expiry(uint64_t age = -1) const;

      /// Return the current expiry for frames of this ID
      uint64_t expiry() const;

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
         * Set to -1 (the default) to never expire frames.
         *
         * Note: if a timestamp -1 frame is saved and this is not -1, all
         * other frames will expire immediately.
         *
         * @return previous default expiry
         **/
      static uint64_t default_expiry(uint64_t age);

      /// Return the default expiry for new frame IDs
      static uint64_t default_expiry();

      /// Return the default prefix for load/save operations
      /// @return std::string holding ".gams.frames"
      static const std::string &default_prefix();

      /**
       * Test if frame is interpolated.
       *
       * @eturn true if this frame was interpolated from two stored frames,
       *        false otherwise
       **/
      bool interpolated() const;

      /**
       * Save this ReferenceFrame to the knowledge base,
       * The saved frames will be marked with their timestamp for later
       * retrieval. If timestamp is -1, it will always be treated as the most
       * recent frame.
       *
       * @param kb the KnowledgeBase to store into
       **/
      void save(madara::knowledge::KnowledgeBase &kb,
                std::string prefix = default_prefix()) const;

      /**
       * Save this ReferenceFrame to the knowledge base,
       * The saved frames will be marked with their timestamp for later
       * retrieval. If timestamp is -1, it will always be treated as the most
       * recent frame.
       *
       * @param kb the KnowledgeBase to store into
       * @param expiry use this expiry time instead of the one set on this ID
       **/
      void save(madara::knowledge::KnowledgeBase &kb,
                uint64_t expiry,
                std::string prefix = default_prefix()) const;

      /**
       * Load a single ReferenceFrame, by ID.
       *
       * @param id the ID of the frame to load
       * @param timestamp if -1, gets the latest frame (no interpolation)
       *   Otherwise, gets the frame at a specified timestamp,
       *   interpolated necessary.
       *
       * @return the imported ReferenceFrame, or an invalid frame if none
       *         exists.
       **/
      static ReferenceFrame load(
              madara::knowledge::KnowledgeBase &kb,
              const std::string &id,
              uint64_t timestamp = -1,
              std::string prefix = default_prefix());

      /**
       * Load ReferenceFrames, by ID, and their common ancestors. Will
       * interpolate frames to ensure the returned frames all have a common
       * timestamp.
       *
       * @tparam an InputIterator, of item type std::string
       *
       * @param begin beginning iterator
       * @param end ending iterator
       * @param timestamp if -1, the latest possible tree will be returned.
       *   Otherwise, the specified timestamp will be returned.
       *
       * @return a vector of ReferenceFrames, each corresponding to the
       *   input IDs, in the same order. If the timestamp specified cannot
       *   be satisfied, returns an empty vector.
       **/
      template<typename InputIterator>
      static std::vector<ReferenceFrame> load_tree(
            madara::knowledge::KnowledgeBase &kb,
            InputIterator begin,
            InputIterator end,
            uint64_t timestamp = -1,
            std::string prefix = default_prefix());

      /**
       * Load ReferenceFrames, by ID, and their common ancestors. Will
       * interpolate frames to ensure the returned frames all have a common
       * timestamp.
       *
       * @tparam a Container, supporting cbegin() and cend(),
       *    of item type std::string
       *
       * @param ids a Container of ids
       * @param timestamp if -1, the latest possible tree will be returned.
       *   Otherwise, the specified timestamp will be returned.
       *
       * @return a vector of ReferenceFrames, each corresponding to the
       *   input IDs, in the same order. If the timestamp specified cannot
       *   be satisfied, returns an empty vector.
       **/
      template<typename Container>
      static std::vector<ReferenceFrame> load_tree(
            madara::knowledge::KnowledgeBase &kb,
            const Container &ids,
            uint64_t timestamp = -1,
            std::string prefix = default_prefix());

      /**
       * Save this ReferenceFrame to the knowledge base, with a specific key
       * value.
       *
       * @param kb the KnowledgeBase to save to
       * @param key a key prefix to save with
       **/
      void save_as(
            madara::knowledge::KnowledgeBase &kb,
            const std::string &key,
            const std::string &prefix) const;

      /**
       * Save this ReferenceFrame to the knowledge base,
       * with a specific key value.
       *
       * @param kb the KnowledgeBase to save to
       * @param key a key prefix to save with
       * @param expiry use this expiry time instead of the one set on this ID
       **/
      void save_as(madara::knowledge::KnowledgeBase &kb,
                   const std::string &key, uint64_t expiry,
                   const std::string &prefix) const;

      /**
       * Interpolate a frame between the given frame; use the given parent.
       * Note: no sanity checking is done. Ensure that parent has a compatible
       * timestamp, and that this and other are the same frame at different
       * times. Users should generally not call this directly. Use load() or
       * load_tree() instead.
       *
       * @param other the other frame to interpolate towards.
       * @param parent the parent the returned frame will have.
       * @param time the timestamp to interpolate at.
       * @return the interpolated frame.
       **/
      ReferenceFrame interpolate(const ReferenceFrame &other,
          ReferenceFrame parent, uint64_t time) const;

      friend class ReferenceFrameVersion;
    };

    /**
     * For internal use. Typical users should not create objects of this type.
     *
     * Stores information and translation functions for various frame types,
     * such as Cartesian and GPS.
     **/
    struct GAMS_EXPORT ReferenceFrameType {
      /**
       * The type's ID
       **/
      int type_id;

      /**
       * A human-readable name for reference frame types
       **/
      const char *name;

      void (*transform_linear_to_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z);

      void (*transform_linear_from_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z);

      void (*normalize_linear)(
                      const ReferenceFrameType *self,
                      double &x, double &y, double &z);

      double (*calc_distance)(
                      const ReferenceFrameType *self,
                      double x1, double y1, double z1,
                      double x2, double y2, double z2);

      void (*transform_angular_to_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double orx, double ory, double orz,
                      double &rx, double &ry, double &rz);

      void (*transform_angular_from_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double orx, double ory, double orz,
                      double &rx, double &ry, double &rz);

      void (*normalize_angular)(
                      const ReferenceFrameType *self,
                      double &rx, double &ry, double &rz);

      double (*calc_angle)(
                      const ReferenceFrameType *self,
                      double rx1, double ry1, double rz1,
                      double rx2, double ry2, double rz2);

      void (*transform_pose_to_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz);

      void (*transform_pose_from_origin)(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz);

      void (*normalize_pose)(
                      const ReferenceFrameType *self,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz);
    };

    /**
     * Default frame. Cartesian type. Will be used by any Coordinates
     * by default, if no other is specified. In general, you should not
     * need to create any other parent-less Cartesian frames.
     **/
    GAMS_EXPORT const ReferenceFrame &default_frame (void);
  }
}

#endif
