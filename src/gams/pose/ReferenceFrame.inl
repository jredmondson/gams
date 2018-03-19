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

namespace gams
{
  namespace pose
  {
#ifdef CPP11
    inline ReferenceFrame::ReferenceFrame()
      : extern_origin_(false)
    {
      new (&origin_) Pose(*this, 0, 0, 0, 0, 0, 0);
    }

    inline ReferenceFrame::ReferenceFrame(const Pose &origin)
      : extern_origin_(false)
    {
      new (&origin_) Pose(origin);
    }

    inline ReferenceFrame::ReferenceFrame(const Pose *origin)
      : ptr_origin_(origin),
        extern_origin_(true) {}

    inline ReferenceFrame::ReferenceFrame(const ReferenceFrame &o)
      : extern_origin_(o.origin_is_external())
    {
      if(extern_origin_)
        ptr_origin_ = o.ptr_origin_;
      else
        new (&origin_) Pose(o.origin_);
    }

    inline void ReferenceFrame::operator=(const ReferenceFrame &o)
    {
      extern_origin_ = o.origin_is_external();
      if(extern_origin_)
        ptr_origin_ = o.ptr_origin_;
      else
        new (&origin_) Pose(o.origin_);
    }

    inline ReferenceFrame::~ReferenceFrame()
    {
      if(!extern_origin_)
        origin_.~Pose();
    }

    inline const Pose &ReferenceFrame::origin() const
    {
      return extern_origin_ ? *ptr_origin_ : origin_;
    }

    inline const Pose &ReferenceFrame::origin(const Pose &new_origin)
    {
      if(!extern_origin_)
        origin_.~Pose();
      extern_origin_ = false;
      new (&origin_) Pose(new_origin);
      return origin_;
    }

    inline void ReferenceFrame::bind_origin(const Pose *new_origin)
    {
      if(!extern_origin_)
        origin_.~Pose();
      extern_origin_ = true;
      ptr_origin_ = new_origin;
    }
#else
    inline ReferenceFrame::ReferenceFrame()
      : origin_(new Pose(*this, 0, 0, 0, 0, 0, 0)),
        extern_origin_(false) {}

    inline ReferenceFrame::ReferenceFrame(const Pose &origin)
      : origin_(new Pose(origin)),
        extern_origin_(false) {}

    inline ReferenceFrame::ReferenceFrame(const Pose *origin)
      : origin_(origin),
        extern_origin_(true) {}

    inline ReferenceFrame::ReferenceFrame(const ReferenceFrame &o)
      : extern_origin_(o.origin_is_external())
    {
      if(extern_origin_)
        origin_ = o.origin_;
      else
        origin_ = new Pose(*o.origin_);
    }

    inline void ReferenceFrame::operator=(
      const ReferenceFrame &o)
    {
      extern_origin_ = o.origin_is_external();

      if(extern_origin_)
        origin_ = o.origin_;
      else
        origin_ = new Pose(*o.origin_);
    }

    inline ReferenceFrame::~ReferenceFrame()
    {
      if(!extern_origin_)
      {
        delete origin_;
      }
    }

    inline const Pose &ReferenceFrame::origin() const
    {
      return *origin_;
    }

    inline const Pose &ReferenceFrame::origin(const Pose &new_origin)
    {
      if(!extern_origin_)
        delete origin_;
      origin_ = nullptr;
      extern_origin_ = false;
      return *(origin_ = new Pose(new_origin));
    }

    inline void ReferenceFrame::bind_origin(const Pose *new_origin)
    {
      if(!extern_origin_)
        delete origin_;
      origin_ = nullptr;
      extern_origin_ = true;
      origin_ = new_origin;
    }
#endif

    inline const ReferenceFrame &ReferenceFrame::origin_frame() const {
      return origin().frame();
    }

    inline bool ReferenceFrame::operator==(const ReferenceFrame &other) const
    {
      return this == &other;
    }

    inline bool ReferenceFrame::operator!=(const ReferenceFrame &other) const
    {
      return !(*this == other);
    }

    inline std::string ReferenceFrame::name() const
    {
      return get_name();
    }

    template<typename CoordType>
    inline void ReferenceFrame::transform(
              CoordType &in, const ReferenceFrame &to_frame)
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

    template<typename CoordType>
    inline void ReferenceFrame::transform_other(
                                CoordType &in, const ReferenceFrame &to_frame)
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

    template<typename CoordType>
    inline const ReferenceFrame &ReferenceFrame::common_parent_transform(
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

    template<typename CoordType>
    inline const ReferenceFrame &
            ReferenceFrame::common_parent_transform_other(
                                        CoordType &in1, CoordType &in2)
    {
      const ReferenceFrame *common_parent =
                     find_common_frame(&in1.frame(), &in2.frame());

      if(common_parent == nullptr)
        throw unrelated_frames(in1.frame(), in2.frame());
      else
      {
        std::cout << common_parent->origin() << std::endl;
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

    template<typename CoordType>
    inline double ReferenceFrame::distance(
                    const CoordType &coord1, const CoordType &coord2)
    {
      if (coord1.frame() == coord2.frame())
      {
        return calc_difference(coord1, coord2);
      }
      else
      {
        CoordType coord1_conv(coord1);
        CoordType coord2_conv(coord2);
        const ReferenceFrame &pframe =
             common_parent_transform(coord1_conv, coord2_conv);

        return calc_difference(coord1_conv, coord2_conv);
      }
    }


    template<typename CoordType>
    inline void ReferenceFrame::transform_to_origin(CoordType &in)
    {
      static_assert(sizeof(in)<0, "must specialize ReferenceFrame::transform_to_origin for this coordinate type");
    }

    template<typename CoordType>
    inline void ReferenceFrame::transform_from_origin(
            CoordType &in, const ReferenceFrame &to_frame)
    {
      static_assert(sizeof(in)<0, "must specialize ReferenceFrame::transform_from_origin for this coordinate type");
    }

    template<typename CoordType>
    inline double ReferenceFrame::calc_difference(const CoordType &in1,
                                                   const CoordType &in2)
    {
      static_assert(sizeof(in1)<0, "must specialize ReferenceFrame::calc_difference for this coordinate type");
    }

    template<typename CoordType>
    inline void ReferenceFrame::normalize(CoordType &in)
    {
      static_assert(sizeof(in)<0, "must specialize ReferenceFrame::normalize for this coordinate type");
    }

    template<>
    inline void ReferenceFrame::transform_to_origin<>(Linear &in)
    {
      in.frame().transform_linear_to_origin(in.x_, in.y_, in.z_);
    }

    template<>
    inline void ReferenceFrame::transform_to_origin<>(Angular &in)
    {
      in.frame().transform_angular_to_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void ReferenceFrame::transform_to_origin<>(Pose &in)
    {
      in.frame().transform_pose_to_origin(in.x_, in.y_, in.z_,
                                          in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void ReferenceFrame::transform_from_origin<>(
        Linear &in, const ReferenceFrame &to_frame)
    {
      to_frame.transform_linear_from_origin(in.x_, in.y_, in.z_);
    }

    template<>
    inline void ReferenceFrame::transform_from_origin<>(
        Angular &in, const ReferenceFrame &to_frame)
    {
      to_frame.transform_angular_from_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void ReferenceFrame::transform_from_origin<>(
      Pose &in, const ReferenceFrame &to_frame)
    {
      to_frame.transform_pose_from_origin(in.x_, in.y_, in.z_,
                                          in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline double ReferenceFrame::calc_difference<>(
        const Linear &loc1, const Linear &loc2)
    {
      return loc1.frame().calc_distance(loc1.x_, loc1.y_, loc1.z_,
                                        loc2.x_, loc2.y_, loc2.z_);
    }

    template<>
    inline double ReferenceFrame::calc_difference<>(
        const Angular &rot1, const Angular &rot2)
    {
      return rot1.frame().calc_angle(rot1.rx_, rot1.ry_, rot1.rz_,
                                     rot2.rx_, rot2.ry_, rot2.rz_);
    }

    template<>
    inline double ReferenceFrame::calc_difference<>(
          const Pose &pose1, const Pose &pose2)
    {
      return pose1.frame().calc_distance(pose1.x_, pose1.y_, pose1.z_,
                                         pose2.x_, pose2.y_, pose2.z_);
    }

    inline void ReferenceFrame::normalize_linear(
                                      double &x, double &y, double &z) const
    {
      do_normalize_linear(x, y, z);
    }

    inline void ReferenceFrame::normalize_angular(
                                      double &rx, double &ry, double &rz) const
    {
      do_normalize_angular(rx, ry, rz);
    }

    inline void ReferenceFrame::normalize_pose(
                                      double &x, double &y, double &z,
                                      double &rx, double &ry, double &rz) const
    {
      do_normalize_pose(x, y, z, rx, ry, rz);
    }

    template<>
    inline void ReferenceFrame::normalize<>(Linear &loc)
    {
      loc.frame().normalize_linear(loc.x_, loc.y_, loc.z_);
    }

    template<>
    inline void ReferenceFrame::normalize<>(Angular &rot)
    {
      rot.frame().normalize_angular(rot.rx_, rot.ry_, rot.rz_);
    }

    template<>
    inline void ReferenceFrame::normalize<>(Pose &pose)
    {
      pose.frame().normalize_pose(pose.x_, pose.y_, pose.z_,
                                  pose.rx_, pose.ry_, pose.rz_);
    }

// Make Visual Studio Intellisense ignore these definitions; it gets confused
// and thinks they're declarations for some reason.
#ifndef __INTELLISENSE__

    template<typename CoordType>
    inline CoordType Coordinate<CoordType>::transform_to(
                                const ReferenceFrame &new_frame) const
    {
      CoordType ret(as_coord_type());
      ReferenceFrame::transform(ret, new_frame);
      return ret;
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::transform_this_to(
                                            const ReferenceFrame &new_frame)
    {
      ReferenceFrame::transform(as_coord_type(), new_frame);
    }

    template<typename CoordType>
    inline double Coordinate<CoordType>::distance_to(
                                                const CoordType &target) const
    {
      return ReferenceFrame::distance(as_coord_type(), target.as_coord_type());
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::normalize()
    {
      frame().normalize(as_coord_type());
    }

#endif // ifndef __INTELLISENSE__

    inline std::ostream &operator<<(std::ostream &o, const Linear &loc)
    {
      o << loc.frame().name() << "Linear" << loc.as_vec();
      return o;
    }

    inline std::ostream &operator<<(std::ostream &o, const Angular &rot)
    {
      o << rot.frame().name() << "Angular" << rot.as_vec();
      return o;
    }

    inline std::ostream &operator<<(std::ostream &o, const Pose &pose)
    {
      o << pose.frame().name() << "Pose" << pose.as_vec();
      return o;
    }


    inline SimpleRotateFrame::SimpleRotateFrame() : ReferenceFrame() {}

    inline SimpleRotateFrame::SimpleRotateFrame(const Pose &origin)
      : ReferenceFrame(origin) {}

    inline SimpleRotateFrame::SimpleRotateFrame(Pose *origin)
      : ReferenceFrame(origin) {}


    inline unrelated_frames::unrelated_frames(const ReferenceFrame &from_frame,
      const ReferenceFrame &to_frame) :
      std::runtime_error("No transform path found between frames."),
      from_frame(from_frame), to_frame(to_frame) {}

    inline unrelated_frames::~unrelated_frames() throw() {}


    template<typename CoordType>
    inline bad_coord_type<CoordType>::bad_coord_type(
                    const ReferenceFrame &frame, const std::string &fn_name)
      : std::runtime_error("Coordinate type " + CoordType::name() +
            " not supported by frame type " + frame.name() +
            " for operation " + fn_name + "."),
        coord_type_name(CoordType::name()),
        fn_name(fn_name), frame(frame) {}

    template<typename CoordType>
    inline bad_coord_type<CoordType>::~bad_coord_type() throw() {}


    inline undefined_transform::undefined_transform(
            const ReferenceFrame &parent_frame,
            const ReferenceFrame &child_frame,
            bool is_child_to_parent, bool unsupported_angular)
      : std::runtime_error(is_child_to_parent ?
            ("No defined transform from embedded " + child_frame.name() +
              " frame to parent " + parent_frame.name() + " frame")
          : ("No defined transform from parent " + parent_frame.name() +
              " frame to embedded " + child_frame.name() + " frame") +
            (unsupported_angular ? " involving angular." : ".")),
        parent_frame(parent_frame), child_frame(child_frame),
          is_child_to_parent(is_child_to_parent),
          unsupported_angular(unsupported_angular) {}

    inline undefined_transform::~undefined_transform() throw() {}
  }
}

#endif
