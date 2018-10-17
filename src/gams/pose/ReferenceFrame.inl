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
 * @file ReferenceFrame.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference frame class's inline functions
 **/

#ifndef _GAMS_POSE_REFERENCE_FRAME_INL_
#define _GAMS_POSE_REFERENCE_FRAME_INL_

#include "ReferenceFrame.h"

#include "gams/pose/Pose.h"
#include "gams/pose/Velocity.h"
#include "gams/pose/Acceleration.h"
#include "gams/pose/AngularVelocity.h"
#include "gams/pose/AngularAcceleration.h"

namespace gams { namespace pose {

/// Implementation details
namespace impl
{
  template<class C, class Func>
  inline void to_origin(C &in, Func func) {
    ReferenceFrame self_frame = in.frame();
    if (!self_frame.valid()) {
      return;
    }
    const Pose &origin = self_frame.origin();
    ReferenceFrame origin_frame = origin.frame();
    if (origin_frame.valid() && self_frame != origin_frame) {
      const ReferenceFrameType *s = self_frame.type();
      const ReferenceFrameType *o = origin_frame.type();
      //std::cerr << "Transform from " << in.frame().id() << " to " << in.frame().origin_frame().id() << std::endl;
      func(s, o, origin, in);
    }
  }
}

template<typename T>
inline auto transform_to_origin(T &in) ->
  typename std::enable_if<T::positional()>::type
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          T &in) {
      s->transform_linear_to_origin(o, s,
          origin.x(), origin.y(), origin.z(),
          origin.rx(), origin.ry(), origin.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2],
          T::fixed());
    });
}

template<typename T>
inline auto transform_to_origin(T &in) ->
  typename std::enable_if<T::rotational()>::type
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          T &in) {
      s->transform_angular_to_origin(o, s,
          origin.rx(), origin.ry(), origin.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}

inline void transform_to_origin(Pose &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          Pose &in) {
      s->transform_pose_to_origin(o, s,
          origin.x(), origin.y(), origin.z(),
          origin.rx(), origin.ry(), origin.rz(),
          in.pos_vec()[0], in.pos_vec()[1], in.pos_vec()[2],
          in.ori_vec()[0], in.ori_vec()[1], in.ori_vec()[2],
          true);
    });
}

/*
inline void transform_to_origin(StampedPosition &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          StampedPosition &in) {
      s->transform_linear_to_origin(o, s,
          origin.x(), origin.y(), origin.z(),
          origin.rx(), origin.ry(), origin.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}

inline void transform_to_origin(StampedOrientation &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          StampedOrientation &in) {
      s->transform_angular_to_origin(o, s,
          origin.rx(), origin.ry(), origin.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}*/

inline void transform_to_origin(StampedPose &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          StampedPose &in) {
      s->transform_pose_to_origin(o, s,
          origin.x(), origin.y(), origin.z(),
          origin.rx(), origin.ry(), origin.rz(),
          in.pos_vec()[0], in.pos_vec()[1], in.pos_vec()[2],
          in.ori_vec()[0], in.ori_vec()[1], in.ori_vec()[2],
          true);
    });
}

namespace impl
{
  template<class C, class Func>
  inline void from_origin(C &in, const ReferenceFrame &to_frame, Func func) {
    ReferenceFrame from_frame = in.frame();
    if (!to_frame.valid()) {
      return;
    }
    if (from_frame.valid() && from_frame != to_frame) {
      //std::cerr << "Transform from " << in.frame().id() << " to " << to_frame.id() << std::endl;
      const Pose &to = to_frame.origin();
      const ReferenceFrameType *t = to_frame.type();
      const ReferenceFrameType *f = from_frame.type();
      func(f, t, to, in);
    }
  }
}

template<typename T>
inline auto transform_from_origin(T &in, const ReferenceFrame &to_frame) ->
  typename std::enable_if<T::positional()>::type
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          T &in) {
      f->transform_linear_from_origin(t, f,
          to.x(), to.y(), to.z(),
          to.rx(), to.ry(), to.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2],
          T::fixed());
    });
}


template<typename T>
inline auto transform_from_origin(T &in, const ReferenceFrame &to_frame) ->
  typename std::enable_if<T::rotational()>::type
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          T &in) {
      f->transform_angular_from_origin(t, f,
          to.rx(), to.ry(), to.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}

inline void transform_from_origin(
  Pose &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          Pose &in) {
      f->transform_pose_from_origin(t, f,
          to.x(), to.y(), to.z(),
          to.rx(), to.ry(), to.rz(),
          in.pos_vec()[0], in.pos_vec()[1], in.pos_vec()[2],
          in.ori_vec()[0], in.ori_vec()[1], in.ori_vec()[2],
          true);
    });
}

/*
inline void transform_from_origin(
    StampedPosition &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          StampedPosition &in) {
      f->transform_linear_from_origin(t, f,
          to.x(), to.y(), to.z(),
          to.rx(), to.ry(), to.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}


inline void transform_from_origin(
    StampedOrientation &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          StampedOrientation &in) {
      f->transform_angular_from_origin(t, f,
          to.rx(), to.ry(), to.rz(),
          in.vec()[0], in.vec()[1], in.vec()[2]);
    });
}
*/

inline void transform_from_origin(
  StampedPose &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          StampedPose &in) {
      f->transform_pose_from_origin(t, f,
          to.x(), to.y(), to.z(),
          to.rx(), to.ry(), to.rz(),
          in.pos_vec()[0], in.pos_vec()[1], in.pos_vec()[2],
          in.ori_vec()[0], in.ori_vec()[1], in.ori_vec()[2],
          true);
    });
}

inline double difference(
    const Position &loc1, const Position &loc2)
{
  const ReferenceFrameType *ft = loc1.frame().type();
  return ft->calc_distance(ft, loc1.x(), loc1.y(), loc1.z(),
                               loc2.x(), loc2.y(), loc2.z());
}

inline double difference(
    const Orientation &rot1, const Orientation &rot2)
{
  const ReferenceFrameType *ft = rot1.frame().type();
  return ft->calc_angle(ft, rot1.rx(), rot1.ry(), rot1.rz(),
                            rot2.rx(), rot2.ry(), rot2.rz());
}

inline double difference(
      const Pose &pose1, const Pose &pose2)
{
  const ReferenceFrameType *ft = pose1.frame().type();
  return ft->calc_distance(ft, pose1.x(), pose1.y(), pose1.z(),
                               pose2.x(), pose2.y(), pose2.z());
}

inline double difference(
    const StampedPosition &loc1, const StampedPosition &loc2)
{
  const ReferenceFrameType *ft = loc1.frame().type();
  return ft->calc_distance(ft, loc1.x(), loc1.y(), loc1.z(),
                               loc2.x(), loc2.y(), loc2.z());
}

inline double difference(
    const StampedOrientation &rot1, const StampedOrientation &rot2)
{
  const ReferenceFrameType *ft = rot1.frame().type();
  return ft->calc_angle(ft, rot1.rx(), rot1.ry(), rot1.rz(),
                            rot2.rx(), rot2.ry(), rot2.rz());
}

inline double difference(
      const StampedPose &pose1, const StampedPose &pose2)
{
  const ReferenceFrameType *ft = pose1.frame().type();
  return ft->calc_distance(ft, pose1.x(), pose1.y(), pose1.z(),
                               pose2.x(), pose2.y(), pose2.z());
}

inline void normalize(Position &loc)
{
  const ReferenceFrameType *ft = loc.frame().type();
  ft->normalize_linear(ft, loc.vec()[0], loc.vec()[1], loc.vec()[2]);
}

inline void normalize(Orientation &rot)
{
  const ReferenceFrameType *ft = rot.frame().type();
  ft->normalize_angular(ft, rot.vec()[0], rot.vec()[1], rot.vec()[2]);
}

inline void normalize(Pose &pose)
{
  const ReferenceFrameType *ft = pose.frame().type();
  ft->normalize_pose(ft, pose.pos_vec()[0], pose.pos_vec()[1], pose.pos_vec()[2],
                         pose.ori_vec()[0], pose.ori_vec()[1], pose.ori_vec()[2]);
}

inline void normalize(StampedPosition &loc)
{
  const ReferenceFrameType *ft = loc.frame().type();
  ft->normalize_linear(ft, loc.vec()[0], loc.vec()[1], loc.vec()[2]);
}

inline void normalize(StampedOrientation &rot)
{
  const ReferenceFrameType *ft = rot.frame().type();
  ft->normalize_angular(ft, rot.vec()[0], rot.vec()[1], rot.vec()[2]);
}

inline void normalize(StampedPose &pose)
{
  const ReferenceFrameType *ft = pose.frame().type();
  ft->normalize_pose(ft, pose.pos_vec()[0], pose.pos_vec()[1], pose.pos_vec()[2],
                         pose.ori_vec()[0], pose.ori_vec()[1], pose.ori_vec()[2]);
}

inline bool ReferenceFrameVersion::operator==(const ReferenceFrame &other) const
{
  return operator==(*other.impl_);
}

inline bool ReferenceFrameVersion::operator!=(const ReferenceFrame &other) const
{
  return !(*this == other);
}

inline bool ReferenceFrameVersion::operator==(const ReferenceFrameVersion &other) const
{
  return this == &other;
}

inline bool ReferenceFrameVersion::operator!=(const ReferenceFrameVersion &other) const
{
  return !(*this == other);
}

/**
 * Transform coordinate from its current from, to the specified frame
 * This transformation is in-place (modifies the in parameters.
 * Called by the transform_to member function of Coordinates
 *
 * @tparam CoordType the type of Coordinate (e.g., Pose, Position)
 * @param in the Coordinate to transform. This object may be modified.
 * @param to_frame the frame to transform into
 *
 * @throws unrelated_frame if no common parent.
 **/
template<typename CoordType>
inline void transform(
          CoordType &in,
          const ReferenceFrame &to_frame)
{
  if (to_frame == in.frame()) {
    return;
  }
  if (!to_frame.valid()) {
    throw unrelated_frames(in.frame(), to_frame);
  }
  if (!in.frame().valid()) {
    throw unrelated_frames(in.frame(), to_frame);
  }
  if (to_frame == in.frame().origin_frame())
  {
    transform_to_origin(in);
    in.frame(in.frame().origin_frame());
  }
  else if (to_frame.origin_frame() == in.frame())
  {
    transform_from_origin(in, to_frame);
    in.frame(to_frame);
  }
  else
    transform_other(in, to_frame);
}

/**
 * Transform into another frame, if coordinates are not directly related.
 *
 * @tparam CoordType the type of Coordinate (e.g., Pose, Position)
 * @param in the coordinate to transform (in-place)
 * @param to_frame the frame to transform into
 *
 * @throws unrelated_frame if no common parent.
 **/
template<typename CoordType>
inline void transform_other(
              CoordType &in,
              const ReferenceFrame &to_frame)
{
  std::vector<const ReferenceFrame *> to_stack;
  const ReferenceFrame *transform_via =
                      find_common_frame(&in.frame(), &to_frame, &to_stack);

  if(transform_via == nullptr)
    throw unrelated_frames(in.frame(), to_frame);
  else
  {
    while(in.frame() != *transform_via)
    {
      transform_to_origin(in);
      in.frame(in.frame().origin_frame());
    }
    while(in.frame() != to_frame && !to_stack.empty())
    {
      transform_from_origin(in, *to_stack.back());
      in.frame(*to_stack.back());
      to_stack.pop_back();
    }
  }
}

/**
 * Transform input coordinates into their common parent. If no common
 * parent exists, throws, unrelated_frames exception
 *
 * @tparam CoordType the type of Coordinate (e.g., Pose, Position)
 * @param in1 the frame to transform from
 * @param in2 the frame to transform into
 * @return the common frame the coordinates were transformed to.
 *
 * @throws unrelated_frame if no common parent.
 **/
template<typename CoordType>
inline ReferenceFrame common_parent_transform(
                   CoordType &in1, CoordType &in2)
{
  if(in1.frame() == in2.frame())
    return in1.frame();
  else if (in1.frame() == in2.frame().origin_frame())
  {
    transform_to_origin(in2);
    return in2.frame(in1.frame());
  }
  else if (in2.frame() == in1.frame().origin_frame())
  {
    transform_to_origin(in1);
    return in1.frame(in2.frame());
  }
  else
    return common_parent_transform_other(in1, in2);
}

/**
 * Transform into common parent, if coordinates are not directly related.
 *
 * @tparam CoordType the type of Coordinate (e.g., Pose, Position)
 * @param in1 the coordinate to transform (in place)
 * @param in2 the other coordinate to transform (in place)
 * @return the common frame the coordinates were transformed to.
 *
 * @throws unrelated_frame if no common parent.
 **/
template<typename CoordType>
inline ReferenceFrame common_parent_transform_other(
                     CoordType &in1, CoordType &in2)
{
  const ReferenceFrame *common_parent =
                 find_common_frame(&in1.frame(), &in2.frame());

  if(common_parent == nullptr)
    throw unrelated_frames(in1.frame(), in2.frame());
  else
  {
    while(in1.frame() != *common_parent)
    {
      transform_to_origin(in1);
      in1.frame(in1.frame().origin_frame());
    }
    while(in2.frame() != *common_parent)
    {
      transform_to_origin(in2);
      in2.frame(in2.frame().origin_frame());
    }
  }
  return *common_parent;
}

/**
 * Calculate distances between coordinates. If they have different
 * frames, first transform to their lowest common parent
 * Called by the distance_to member function of Coordinates
 *
 * @tparam CoordType the type of Coordinate (e.g., Pose, Position)
 * @param coord1 The coordinate to measure from
 * @param coord2 The coordinate to measure to
 * @return the distance, in meters, between the coordinates
 **/
template<typename CoordType>
inline double distance(
           const CoordType &coord1,
           const CoordType &coord2)
{
  if (coord1.frame() == coord2.frame())
  {
    return difference(coord1, coord2);
  }
  else
  {
    CoordType coord1_conv(coord1);
    CoordType coord2_conv(coord2);
    common_parent_transform(coord1_conv, coord2_conv);

    return difference(coord1_conv, coord2_conv);
  }
}

// Make Visual Studio Intellisense ignore these definitions; it gets confused
// and thinks they're declarations for some reason.
#ifndef __INTELLISENSE__

/*
template<typename CoordType>
inline CoordType Coordinate<CoordType>::transform_to(
                        const ReferenceFrame &new_frame) const
{
  CoordType ret(as_coord_type());
  transform(ret, new_frame);
  return ret;
}

template<typename CoordType>
inline void Coordinate<CoordType>::transform_this_to(
                        const ReferenceFrame &new_frame)
{
  transform(as_coord_type(), new_frame);
}

template<typename CoordType>
inline double Coordinate<CoordType>::distance_to(
                        const CoordType &target) const
{
  return distance(as_coord_type(), target.as_coord_type());
}

template<typename CoordType>
inline void Coordinate<CoordType>::normalize()
{
  pose::normalize(as_coord_type());
}
*/

template<typename CoordType>
inline auto Framed<CoordType>::transform_to(
                  const ReferenceFrame &new_frame) const ->
  typename Framed::derived_type
{
  auto ret = self();
  transform(ret, new_frame);
  return ret;
}

template<typename CoordType>
inline void Framed<CoordType>::transform_this_to(
                        const ReferenceFrame &new_frame)
{
  transform(self(), new_frame);
}

template<typename CoordType>
inline double Framed<CoordType>::distance_to(
                const typename Framed::derived_type &target) const
{
  return distance(self(), target.self());
}

template<typename CoordType>
inline void Framed<CoordType>::normalize()
{
  pose::normalize(self());
}

#endif // ifndef __INTELLISENSE__

template<typename Base>
inline std::ostream &operator<<(std::ostream &o, const Framed<Base> &v)
{
  if (v.frame().valid()) {
    o << v.frame().name();
  }
  o << static_cast<const Base &>(v);
  return o;
}

inline const Pose &ReferenceFrame::origin() const {
  return impl_->origin();
}

inline ReferenceFrame ReferenceFrame::pose(
    const Pose &new_origin) const {
  return impl_->pose(new_origin);
}

inline ReferenceFrame ReferenceFrame::move(
    const Position &new_origin) const {
  return impl_->move(new_origin);
}

inline ReferenceFrame ReferenceFrame::orient(
    const Orientation &new_origin) const {
  return impl_->orient(new_origin);
}

inline ReferenceFrame ReferenceFrame::pose(
    const Pose &new_origin,
    uint64_t timestamp) const {
  return impl_->pose(new_origin, timestamp);
}

inline ReferenceFrame ReferenceFrame::move(
    const Position &new_origin,
    uint64_t timestamp) const {
  return impl_->move(new_origin, timestamp);
}

inline ReferenceFrame ReferenceFrame::orient(
    const Orientation &new_origin,
    uint64_t timestamp) const {
  return impl_->orient(new_origin, timestamp);
}

inline ReferenceFrame ReferenceFrame::origin_frame() const {
  return impl_->origin_frame();
}

inline bool ReferenceFrame::operator==(
    const ReferenceFrame &other) const {
  return impl_->operator==(other);
}

inline bool ReferenceFrame::operator!=(
      const ReferenceFrame &other) const {
  return impl_->operator!=(other);
}

inline std::string ReferenceFrame::name() const {
  return impl_->name();
}

inline const ReferenceFrameType *ReferenceFrame::type() const {
  return impl_->type();
}

inline const std::string &ReferenceFrame::id() const {
  return impl_->id();
}

inline uint64_t ReferenceFrame::timestamp() const {
  return impl_->timestamp();
}

inline ReferenceFrame ReferenceFrame::timestamp(uint64_t timestamp) const {
  return impl_->timestamp(timestamp);
}

inline uint64_t ReferenceFrame::expiry(uint64_t age) const {
  return impl_->ident().expiry(age);
}

inline uint64_t ReferenceFrame::expiry() const {
  return impl_->ident().expiry();
}

inline bool ReferenceFrame::interpolated() const {
  return impl_->interpolated();
}

inline bool ReferenceFrame::temp() const {
  return impl_->temp();
}

inline uint64_t ReferenceFrame::default_expiry(uint64_t age) {
  return ReferenceFrameIdentity::default_expiry(age);
}

inline uint64_t ReferenceFrame::default_expiry() {
  return ReferenceFrameIdentity::default_expiry();
}

inline const std::string &ReferenceFrame::default_prefix() {
  return ReferenceFrameIdentity::default_prefix();
}

inline void ReferenceFrame::save(
      madara::knowledge::KnowledgeBase &kb,
      const FrameEvalSettings &settings) const {
  return impl_->save(kb, settings);
}

inline void ReferenceFrame::save(
      madara::knowledge::KnowledgeBase &kb,
      uint64_t expiry,
      const FrameEvalSettings &settings) const {
  return impl_->save(kb, expiry, settings);
}

inline ReferenceFrame ReferenceFrame::load(
        madara::knowledge::KnowledgeBase &kb,
        const std::string &id,
        uint64_t timestamp,
        const FrameEvalSettings &settings) {
  return ReferenceFrameVersion::load(kb, id, timestamp, settings);
}

template<typename InputIterator>
inline std::vector<ReferenceFrame> ReferenceFrame::load_tree(
      madara::knowledge::KnowledgeBase &kb,
      InputIterator begin,
      InputIterator end,
      uint64_t timestamp,
      const FrameEvalSettings &settings) {
  return ReferenceFrameVersion::load_tree(kb, begin, end,
                                          timestamp, settings);
}

template<typename Container>
inline std::vector<ReferenceFrame> ReferenceFrame::load_tree(
      madara::knowledge::KnowledgeBase &kb,
      const Container &ids,
      uint64_t timestamp,
      const FrameEvalSettings &settings) {
  return load_tree(kb, ids.begin(), ids.end(), timestamp, settings);
}

inline std::vector<ReferenceFrame> ReferenceFrame::load_tree(
      madara::knowledge::KnowledgeBase &kb,
      const std::initializer_list<const char *> &ids,
      uint64_t timestamp,
      const FrameEvalSettings &settings) {
  return load_tree(kb, ids.begin(), ids.end(), timestamp, settings);
}

inline void ReferenceFrame::save_as(
      madara::knowledge::KnowledgeBase &kb,
      const std::string &key,
      const FrameEvalSettings &settings) const {
  return impl_->save_as(kb, key, settings);
}

inline void ReferenceFrame::save_as(
      madara::knowledge::KnowledgeBase &kb,
      const std::string &key,
      uint64_t expiry,
      const FrameEvalSettings &settings) const {
  return impl_->save_as(kb, key, expiry, settings);
}

inline ReferenceFrame ReferenceFrame::interpolate(
      const ReferenceFrame &other, ReferenceFrame parent, uint64_t time) const {
  return impl_->interpolate(other, std::move(parent), time);
}

inline unrelated_frames::unrelated_frames(ReferenceFrame from_frame,
  ReferenceFrame to_frame) :
  std::runtime_error("No transform path found between frames."),
  from_frame(from_frame), to_frame(to_frame) {}

inline unrelated_frames::~unrelated_frames() throw() {}

inline undefined_transform::undefined_transform(
        const ReferenceFrameType *parent_frame,
        const ReferenceFrameType *child_frame,
        bool is_child_to_parent, bool unsupported_angular)
  : std::runtime_error(is_child_to_parent ?
      (std::string("No transform from embedded ") + child_frame->name +
        " frame to parent " + parent_frame->name + " frame defined")
    : (std::string("No transform from parent ") + parent_frame->name +
        " frame to embedded " + child_frame->name + " frame defined") +
      (unsupported_angular ? " involving rotation." : ".")),
    parent_frame(parent_frame), child_frame(child_frame),
      is_child_to_parent(is_child_to_parent),
      unsupported_angular(unsupported_angular) {}

inline undefined_transform::~undefined_transform() throw() {}

} }
#endif
