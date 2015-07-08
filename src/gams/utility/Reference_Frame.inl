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
 * @file Reference_Frame.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference frame class's inline functions
 **/

#ifndef _GAMS_UTILITY_REFERENCE_FRAME_INL_
#define _GAMS_UTILITY_REFERENCE_FRAME_INL_

#include <gams/utility/Pose.h>

namespace gams
{
  namespace utility
  {
    inline Reference_Frame::Reference_Frame()
      : origin_(new Pose(*this, 0, 0, 0, 0, 0, 0)),
        destruct_origin_(true) {}

    inline Reference_Frame::Reference_Frame(const Pose &origin)
      : origin_(new Pose(origin)),
        destruct_origin_(true) {}

    inline Reference_Frame::Reference_Frame(Pose *origin)
      : origin_(origin),
        destruct_origin_(false) {}

    inline Reference_Frame::~Reference_Frame()
    {
      if(destruct_origin_)
      {
        delete origin_;
      }
    }

    inline const Pose &Reference_Frame::origin() const
    {
      return *origin_;
    }

    inline const Pose &Reference_Frame::origin(const Pose &new_origin)
    {
      if(destruct_origin_)
        delete origin_;
      origin_ = nullptr;
      destruct_origin_ = true;
      return *(origin_ = new Pose(new_origin));
    }

    inline const Reference_Frame &Reference_Frame::origin_frame() const {
      return origin().frame();
    }

    inline void Reference_Frame::bind_origin(Pose *new_origin)
    {
      if(destruct_origin_)
        delete origin_;
      origin_ = nullptr;
      destruct_origin_ = false;
      origin_ = new_origin;
    }

    inline bool Reference_Frame::operator==(const Reference_Frame &other) const
    {
      return this == &other;
    }

    inline bool Reference_Frame::operator!=(const Reference_Frame &other) const
    {
      return !(*this == other);
    }

    inline std::string Reference_Frame::name() const
    {
      return get_name();
    }

    template<typename CoordType>
    inline void Reference_Frame::transform(
              CoordType &in, const Reference_Frame &to_frame)
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
    inline void Reference_Frame::transform_other(
                                CoordType &in, const Reference_Frame &to_frame)
    {
      std::vector<const Reference_Frame *> to_stack;
      const Reference_Frame *transform_via =
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
    inline const Reference_Frame &Reference_Frame::common_parent_transform(
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
    inline const Reference_Frame &
            Reference_Frame::common_parent_transform_other(
                                        CoordType &in1, CoordType &in2)
    {
      const Reference_Frame *common_parent =
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
    inline double Reference_Frame::distance(
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
        const Reference_Frame &pframe =
             common_parent_transform(coord1_conv, coord2_conv);

        return calc_difference(coord1_conv, coord2_conv);
      }
    }


    template<typename CoordType>
    inline void Reference_Frame::transform_to_origin(CoordType &in)
    {
      static_assert(sizeof(in)<0, "must specialize Reference_Frame::transform_to_origin for this coordinate type");
    }

    template<typename CoordType>
    inline void Reference_Frame::transform_from_origin(
            CoordType &in, const Reference_Frame &to_frame)
    {
      static_assert(sizeof(in)<0, "must specialize Reference_Frame::transform_from_origin for this coordinate type");
    }

    template<typename CoordType>
    inline double Reference_Frame::calc_difference(const CoordType &in1,
                                                   const CoordType &in2)
    {
      static_assert(sizeof(in1)<0, "must specialize Reference_Frame::calc_difference for this coordinate type");
    }

    template<typename CoordType>
    inline void Reference_Frame::normalize(CoordType &in)
    {
      static_assert(sizeof(in)<0, "must specialize Reference_Frame::normalize for this coordinate type");
    }

    template<>
    inline void Reference_Frame::transform_to_origin<>(Location &in)
    {
      in.frame().transform_location_to_origin(in.x_, in.y_, in.z_);
    }

    template<>
    inline void Reference_Frame::transform_to_origin<>(Rotation &in)
    {
      in.frame().transform_rotation_to_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void Reference_Frame::transform_to_origin<>(Pose &in)
    {
      in.frame().transform_location_to_origin(in.x_, in.y_, in.z_);
      in.frame().transform_rotation_to_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void Reference_Frame::transform_from_origin<>(
        Location &in, const Reference_Frame &to_frame)
    {
      to_frame.transform_location_from_origin(in.x_, in.y_, in.z_);
    }

    template<>
    inline void Reference_Frame::transform_from_origin<>(
        Rotation &in, const Reference_Frame &to_frame)
    {
      to_frame.transform_rotation_from_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline void Reference_Frame::transform_from_origin<>(
      Pose &in, const Reference_Frame &to_frame)
    {
      to_frame.transform_location_from_origin(in.x_, in.y_, in.z_);
      to_frame.transform_rotation_from_origin(in.rx_, in.ry_, in.rz_);
    }

    template<>
    inline double Reference_Frame::calc_difference<>(
        const Location &loc1, const Location &loc2)
    {
      return loc1.frame().calc_distance(loc1.x_, loc1.y_, loc1.z_,
                                        loc2.x_, loc2.y_, loc2.z_);
    }

    template<>
    inline double Reference_Frame::calc_difference<>(
        const Rotation &rot1, const Rotation &rot2)
    {
      return rot1.frame().calc_angle(rot1.rx_, rot1.ry_, rot1.rz_,
                                     rot2.rx_, rot2.ry_, rot2.rz_);
    }

    template<>
    inline double Reference_Frame::calc_difference<>(
          const Pose &pose1, const Pose &pose2)
    {
      return pose1.frame().calc_distance(pose1.x_, pose1.y_, pose1.z_,
                                         pose2.x_, pose2.y_, pose2.z_);
    }

    inline void Reference_Frame::normalize_location(
                                      double &x, double &y, double &z) const
    {
      do_normalize_location(x, y, z);
    }

    inline void Reference_Frame::normalize_rotation(
                                      double &rx, double &ry, double &rz) const
    {
      do_normalize_rotation(rx, ry, rz);
    }

    template<>
    inline void Reference_Frame::normalize<>(Location &loc)
    {
      loc.frame().normalize_location(loc.x_, loc.y_, loc.z_);
    }

    template<>
    inline void Reference_Frame::normalize<>(Rotation &rot)
    {
      rot.frame().normalize_rotation(rot.rx_, rot.ry_, rot.rz_);
    }

    template<>
    inline void Reference_Frame::normalize<>(Pose &pose)
    {
      pose.frame().normalize_location(pose.x_, pose.y_, pose.z_);
      pose.frame().normalize_rotation(pose.rx_, pose.ry_, pose.rz_);
    }

    template<typename CoordType>
    inline CoordType Coordinate<CoordType>::transform_to(
                                const Reference_Frame &new_frame) const
    {
      CoordType ret(as_coord_type());
      Reference_Frame::transform(ret, new_frame);
      return ret;
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::transform_this_to(
                                            const Reference_Frame &new_frame)
    {
      Reference_Frame::transform(as_coord_type(), new_frame);
    }

    template<typename CoordType>
    inline double Coordinate<CoordType>::distance_to(
                                                const CoordType &target) const
    {
      return Reference_Frame::distance(as_coord_type(), target.as_coord_type());
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::normalize()
    {
      frame().normalize(as_coord_type());
    }


    inline std::ostream &operator<<(std::ostream &o, const Location &loc)
    {
      o << loc.frame().name() << "Location" << loc.as_vec();
      return o;
    }

    inline std::ostream &operator<<(std::ostream &o, const Rotation &rot)
    {
      o << rot.frame().name() << "Rotation" << rot.as_vec();
      return o;
    }

    inline std::ostream &operator<<(std::ostream &o, const Pose &pose)
    {
      o << pose.frame().name() << "Pose" << pose.as_vec();
      return o;
    }


    inline Axis_Angle_Frame::Axis_Angle_Frame() : Reference_Frame() {}

    inline Axis_Angle_Frame::Axis_Angle_Frame(const Pose &origin)
      : Reference_Frame(origin) {}

    inline Axis_Angle_Frame::Axis_Angle_Frame(Pose *origin)
      : Reference_Frame(origin) {}


    inline unrelated_frames::unrelated_frames(const Reference_Frame &from_frame,
      const Reference_Frame &to_frame) :
      std::runtime_error("No transform path found between frames."),
      from_frame(from_frame), to_frame(to_frame) {}

    inline unrelated_frames::~unrelated_frames() throw() {}


    template<typename CoordType>
    inline bad_coord_type<CoordType>::bad_coord_type(
                    const Reference_Frame &frame, const std::string &fn_name)
      : std::runtime_error("Coordinate type " + CoordType::name() +
            " not supported by frame type " + frame.name() +
            " for operation " + fn_name + "."),
        coord_type_name(CoordType::name()),
        fn_name(fn_name), frame(frame) {}

    template<typename CoordType>
    inline bad_coord_type<CoordType>::~bad_coord_type() throw() {}


    inline undefined_transform::undefined_transform(
            const Reference_Frame &parent_frame,
            const Reference_Frame &child_frame,
            bool is_child_to_parent, bool unsupported_rotation)
      : std::runtime_error(is_child_to_parent ?
            ("No defined transform from embedded " + child_frame.name() +
              " frame to parent " + parent_frame.name() + " frame")
          : ("No defined transform from parent " + parent_frame.name() +
              " frame to embedded " + child_frame.name() + " frame") +
            (unsupported_rotation ? " involving rotation." : ".")),
        parent_frame(parent_frame), child_frame(child_frame),
          is_child_to_parent(is_child_to_parent),
          unsupported_rotation(unsupported_rotation) {}

    inline undefined_transform::~undefined_transform() throw() {}
  }
}

#endif
