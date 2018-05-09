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

#include <gams/pose/Pose.h>
#include <gams/pose/Velocity.h>
#include <gams/pose/Acceleration.h>
#include <gams/pose/AngularVelocity.h>
#include <gams/pose/AngularAcceleration.h>

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

template<typename C>
inline void transform_to_origin(Linear<C> &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          Linear<C> &in) {
      s->transform_linear_to_origin(o, s,
          origin.x(), origin.y(), origin.z(),
          origin.rx(), origin.ry(), origin.rz(),
          in.v_[0], in.v_[1], in.v_[2]);
    });
}

template<typename C>
inline void transform_to_origin(Angular<C> &in)
{
  impl::to_origin(in, [](
          const ReferenceFrameType *s,
          const ReferenceFrameType *o,
          const Pose &origin,
          Angular<C> &in) {
      s->transform_angular_to_origin(o, s,
          origin.rx(), origin.ry(), origin.rz(),
          in.rv_[0], in.rv_[1], in.rv_[2]);
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
          in.v_[0], in.v_[1], in.v_[2],
          in.rv_[0], in.rv_[1], in.rv_[2]);
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

template<typename C>
inline void transform_from_origin(
    Linear<C> &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          Linear<C> &in) {
      f->transform_linear_from_origin(t, f,
          to.x(), to.y(), to.z(),
          to.rx(), to.ry(), to.rz(),
          in.v_[0], in.v_[1], in.v_[2]);
    });
}


template<typename C>
inline void transform_from_origin(
    Angular<C> &in, const ReferenceFrame &to_frame)
{
  impl::from_origin(in, to_frame, [](
          const ReferenceFrameType *t,
          const ReferenceFrameType *f,
          const Pose &to,
          Angular<C> &in) {
      f->transform_angular_from_origin(t, f,
          to.rx(), to.ry(), to.rz(),
          in.rv_[0], in.rv_[1], in.rv_[2]);
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
          in.v_[0], in.v_[1], in.v_[2],
          in.rv_[0], in.rv_[1], in.rv_[2]);
    });
}

template<typename C>
inline double difference(
    const Linear<C> &loc1, const Linear<C> &loc2)
{
  const ReferenceFrameType *ft = loc1.frame().type();
  return ft->calc_distance(ft, loc1.x(), loc1.y(), loc1.z(),
                               loc2.x(), loc2.y(), loc2.z());
}

template<typename C>
inline double difference(
    const Angular<C> &rot1, const Angular<C> &rot2)
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

template<typename C>
inline void normalize(Linear<C> &loc)
{
  const ReferenceFrameType *ft = loc.frame().type();
  ft->normalize_linear(ft, loc.v_[0], loc.v_[1], loc.v_[2]);
}

template<typename C>
inline void normalize(Angular<C> &rot)
{
  const ReferenceFrameType *ft = rot.frame().type();
  ft->normalize_angular(ft, rot.rv_[0], rot.rv_[1], rot.rv_[2]);
}

inline void normalize(Pose &pose)
{
  const ReferenceFrameType *ft = pose.frame().type();
  ft->normalize_pose(ft, pose.v_[0], pose.v_[1], pose.v_[2],
                         pose.rv_[0], pose.rv_[1], pose.rv_[2]);
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
 * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
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
  if (to_frame == in.frame())
    return;
  else if (to_frame == in.frame().origin_frame())
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
 * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
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
 * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
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
 * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
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
 * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
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
    const ReferenceFrame &pframe =
         common_parent_transform(coord1_conv, coord2_conv);

    return difference(coord1_conv, coord2_conv);
  }
}

// Make Visual Studio Intellisense ignore these definitions; it gets confused
// and thinks they're declarations for some reason.
#ifndef __INTELLISENSE__

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

#endif // ifndef __INTELLISENSE__

inline std::ostream &operator<<(std::ostream &o, const Position &loc)
{
  o << loc.frame().name() << "Position" << loc.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o, const Velocity &loc)
{
  o << loc.frame().name() << "Velocity" << loc.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o, const Acceleration &loc)
{
  o << loc.frame().name() << "Acceleration" << loc.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o, const AngularVelocity &loc)
{
  o << loc.frame().name() << "AngularVelocity" << loc.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o,
                                const AngularAcceleration &loc)
{
  o << loc.frame().name() << "AngularAcceleration" << loc.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o, const Orientation &rot)
{
  o << rot.frame().name() << "Orientation" << rot.as_vec();
  return o;
}

inline std::ostream &operator<<(std::ostream &o, const Pose &pose)
{
  o << pose.frame().name() << "Pose" << pose.as_vec();
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

inline std::string ReferenceFrame::key() const {
  return impl_->key();
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

inline uint64_t ReferenceFrame::default_expiry(uint64_t age) {
  return ReferenceFrameIdentity::default_expiry(age);
}

inline uint64_t ReferenceFrame::default_expiry() {
  return ReferenceFrameIdentity::default_expiry();
}

inline void ReferenceFrame::save(
      madara::knowledge::KnowledgeBase &kb) const {
  return impl_->save(kb);
}

inline void ReferenceFrame::save(
      madara::knowledge::KnowledgeBase &kb,
      uint64_t expiry) const {
  return impl_->save(kb, expiry);
}

inline ReferenceFrame ReferenceFrame::load(
        madara::knowledge::KnowledgeBase &kb,
        const std::string &id,
        uint64_t timestamp) {
  return ReferenceFrameVersion::load(kb, id, timestamp);
}

template<typename InputIterator>
inline std::vector<ReferenceFrame> ReferenceFrame::load_tree(
      madara::knowledge::KnowledgeBase &kb,
      InputIterator begin,
      InputIterator end,
      uint64_t timestamp) {
  return ReferenceFrameVersion::load_tree(kb, begin, end, timestamp);
}

template<typename Container>
inline std::vector<ReferenceFrame> ReferenceFrame::load_tree(
      madara::knowledge::KnowledgeBase &kb,
      const Container &ids,
      uint64_t timestamp) {
  return load_tree(kb, ids.cbegin(), ids.cend(), timestamp);
}

inline void ReferenceFrame::save_as(
      madara::knowledge::KnowledgeBase &kb,
      const std::string &key) const {
  return impl_->save_as(kb, key);
}

inline void ReferenceFrame::save_as(
      madara::knowledge::KnowledgeBase &kb,
      const std::string &key,
      uint64_t expiry) const {
  return impl_->save_as(kb, key, expiry);
}

inline ReferenceFrame ReferenceFrame::interpolate(
      const ReferenceFrame &other, ReferenceFrame parent, uint64_t time) const {
  return impl_->interpolate(other, std::move(parent), time);
}

#if 0
inline ReferenceFrame ReferenceFrame::load_as(
      madara::knowledge::KnowledgeBase &kb,
      const std::string &key) {
  return ReferenceFrameVersion::load_as(kb, key);
}
#endif

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
