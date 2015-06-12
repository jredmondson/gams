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
 * @file Base_Frame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_BASE_FRAME_H_
#define _GAMS_UTILITY_BASE_FRAME_H_

#include "gams/GAMS_Export.h"
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>
#include <gams/utility/Coordinates.h>

/**
 * Following statement or block is executed only if the frame `coord` belongs
 * to is of type `type` (i.e., can be dynamic_cast to it). The casted frame
 * is provided as a const reference in `frame_ref`, in scope only within the
 * following statement/block.
 *
 * Warning: this uses a for loop as implementation, so using break or continue
 * inside will produce unexpected results.
 **/
#define GAMS_WITH_FRAME_TYPE(coord, type, frame_ref) \
  for(const type *__with_frame_type_temp_ptr__ = \
            dynamic_cast<const type *>(&coord.get_frame()), \
        &frame_ref = *__with_frame_type_temp_ptr__; \
        __with_frame_type_temp_ptr__; __with_frame_type_temp_ptr__ = NULL)

#define INVAL_COORD DBL_MAX

namespace gams
{
  namespace utility
  {
    /**
     * Thrown when a reference frame function is called with a coordinate
     * type (e.g., Location, Rotation) that frame does not support.
     **/
    template<typename CoordType>
    class bad_coord_type : public std::runtime_error
    {
    public:
      bad_coord_type(const Base_Frame &frame, const std::string &fn_name);
      ~bad_coord_type() throw() {}

      std::string coord_type_name;
      std::string fn_name;
      const Base_Frame &frame;

      typedef CoordType Coord_Type;
    };

    /**
     * Thrown when an an attempt is made to transform between frames
     * that do not belong to the same frame tree.
     **/
    class unrelated_frames : public std::runtime_error
    {
    public:
      unrelated_frames(const Base_Frame &from_frame,
        const Base_Frame &to_frame) :
        std::runtime_error("No transform path found between frames."),
        from_frame(from_frame), to_frame(to_frame) {}
      ~unrelated_frames() throw() {}

      const Base_Frame &from_frame;
      const Base_Frame &to_frame;
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
      undefined_transform(const Base_Frame &parent_frame,
        const Base_Frame &child_frame, bool is_child_to_parent);
      ~undefined_transform() throw() {}

      const Base_Frame &parent_frame;
      const Base_Frame &child_frame;
      bool is_child_to_parent;
    };

    /**
     * Base class for Reference Frames.
     * Inherit from this class if implementing a new reference frame.
     * Otherwise, do not use directly.
     **/
    class GAMS_Export Base_Frame
    {
    public:
      Base_Frame() : origin(Pose(*this, 0, 0, 0, 0, 0, 0)) {}
      Base_Frame(const Pose &origin) : origin(origin) {}
      virtual ~Base_Frame() {}

      Pose origin;

      bool operator==(const Base_Frame &other) const
      {
        return this == &other;
      }

      /**
       * Transform coordinate `in` from its current from, to the specified frame
       **/
      template<typename CoordType>
      static void transform(CoordType &in, const Base_Frame &to_frame)
      {
        if (to_frame == *in.frame)
          return;
        else if (to_frame == *in.frame->origin.frame)
        {
          in.frame->transform_to_origin(in);
          in.set_frame(in.frame->origin.get_frame());
        }
        else if (*to_frame.origin.frame == *in.frame)
        {
          to_frame.transform_from_origin(in);
          in.set_frame(to_frame);
        }
        else
          transform_other(in, to_frame);
      }

      /**
       * Calculate distances between coordinates. If they have different
       * frames, transform coord2 to coord1's frame first.
       **/
      template<typename CoordType>
      static double distance(const CoordType &coord1, const CoordType &coord2)
      {
        if (coord1.get_frame() == coord2.get_frame())
        {
          return calc_distance_within_frame(coord1, coord2, coord1.get_frame());
        }
        else
        {
          CoordType coord2_conv(coord2);
          transform(coord2_conv, *coord1.frame);
          return calc_distance_within_frame(coord1, coord2_conv, coord1.get_frame());
        }
      }

      /**
       * Normalizes any angular values in coord according to the conventions
       * of the frame it belongs to. This is called automatically by any methods
       * that require normalized angles.
       *
       * If adding a new composite coordinate type, specialize this template
       * function accordingly.
       **/
      template<typename CoordType>
      void normalize(CoordType &coord) const
      {
        do_normalize(static_cast<typename CoordType::Base_Type &>(coord));
      }

      virtual std::string get_name() const = 0;

    private:
      /**
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       **/
      template<typename CoordType>
      static double calc_distance_within_frame( const CoordType &coord1,
          const CoordType &coord2, const Base_Frame &frame)
      {
        return frame.calc_distance(coord1, coord2);
      }

      virtual void transform_to_origin(Location_Base &in) const
      {
        throw bad_coord_type<Location>(*this, "transform_to_origin");
      }

      virtual void transform_from_origin(Location_Base &in) const
      {
        throw bad_coord_type<Location>(*this, "transform_from_origin");
      }

      virtual void do_normalize(Location_Base &in) const
      {
      }

      virtual double calc_distance(const Location_Base &loc1, const Location_Base &loc2) const
      {
        throw bad_coord_type<Location>(*this, "calc_distance");
      }

      virtual void transform_to_origin(Rotation_Base &in) const
      {
        throw bad_coord_type<Rotation>(*this, "transform_to_origin");
      }

      virtual void transform_from_origin(Rotation_Base &in) const
      {
        throw bad_coord_type<Rotation>(*this, "transform_from_origin");
      }

      virtual double calc_distance(const Rotation_Base &rot1, const Rotation_Base &rot2) const
      {
        throw bad_coord_type<Rotation>(*this, "calc_distance");
      }

      virtual void do_normalize(Rotation_Base &in) const
      {
      }

      virtual void transform_to_origin(Pose_Base &in) const
      {
        transform_to_origin(static_cast<Location_Base &>(in));
        transform_to_origin(static_cast<Rotation_Base &>(in));
      }

      virtual void transform_from_origin(Pose_Base &in) const
      {
        transform_from_origin(static_cast<Location_Base &>(in));
        transform_from_origin(static_cast<Rotation_Base &>(in));
      }

      static const Base_Frame *find_common_frame(const Base_Frame *from,
        const Base_Frame *to, std::vector<const Base_Frame *> *to_stack);

      template<typename CoordType>
      inline static void transform_other(CoordType &in, const Base_Frame &to_frame);
    };

    template<>
    inline double Base_Frame::calc_distance_within_frame<>(const Pose &pose1,
                      const Pose &pose2, const Base_Frame &frame)
    {
      return frame.calc_distance(static_cast<const Location_Base&>(pose1),
                                 static_cast<const Location_Base&>(pose2));
    }

    template<>
    inline void Base_Frame::normalize<>(Pose &pose) const
    {
      pose.frame->do_normalize(static_cast<Location_Base&>(pose));
      pose.frame->do_normalize(static_cast<Rotation_Base&>(pose));
    }

    template<typename CoordType>
    inline CoordType Frame_Bound<CoordType>::transform_to(const Base_Frame &new_frame) const
    {
      CoordType ret(static_cast<const CoordType &>(*this));
      Base_Frame::transform(ret, new_frame);
      return ret;
    }

    template<typename CoordType>
    inline void Frame_Bound<CoordType>::transform_this_to(const Base_Frame &new_frame)
    {
      Base_Frame::transform(static_cast<CoordType &>(*this), new_frame);
    }

    template<typename CoordType>
    inline double Frame_Bound<CoordType>::distance_to(const Frame_Bound<CoordType> &target) const
    {
      return Base_Frame::distance(static_cast<const CoordType &>(*this),
                                  static_cast<const CoordType &>(target));
    }

    template<typename CoordType>
    inline void Frame_Bound<CoordType>::normalize()
    {
      this->frame->normalize(static_cast<CoordType &>(*this));
    }

    template<typename CoordType>
    inline void Base_Frame::transform_other(CoordType &in, const Base_Frame &to_frame)
    {
      std::vector<const Base_Frame *> to_stack;
      const Base_Frame *transform_via = find_common_frame(in.frame, &to_frame, &to_stack);
      if(transform_via == NULL)
        throw unrelated_frames(in.get_frame(), to_frame);
      else
      {
        const Base_Frame *cur_frame = in.frame;
        while(cur_frame != transform_via)
        {
          cur_frame->transform_to_origin(in);
          in.set_frame(cur_frame->origin.get_frame());
          cur_frame = in.frame;
        }
        while(cur_frame != &to_frame && !to_stack.empty())
        {
          to_stack.back()->transform_from_origin(in);
          in.set_frame(*to_stack.back());
          to_stack.pop_back();
          cur_frame = in.frame;
        }
      }
    }

    inline std::ostream &operator<<(std::ostream &o, const Location &loc)
    {
      o << loc.get_frame().get_name() << "(" << loc.x << "," << loc.y << "," << loc.z << ")";
      return o;
    }

    inline std::ostream &operator<<(std::ostream &o, const Rotation &rot)
    {
      o << rot.get_frame().get_name() << "Rotation(" << rot.rx << "," << rot.ry << "," << rot.rz << ")";
      return o;
    }

    template<typename CoordType>
    inline bad_coord_type<CoordType>
          ::bad_coord_type(const Base_Frame &frame, const std::string &fn_name)
      : std::runtime_error("Coordinate type " + CoordType::get_name() +
            " not supported by frame type " + frame.get_name() +
            " for operation " + fn_name + "."),
        coord_type_name(CoordType::get_name()),
        fn_name(fn_name), frame(frame) {}

    inline undefined_transform::undefined_transform(
            const Base_Frame &parent_frame, const Base_Frame &child_frame,
            bool is_child_to_parent)
      : std::runtime_error(is_child_to_parent ?
            ("No defined transform from embedded " + child_frame.get_name() +
              " frame to parent " + parent_frame.get_name() + " frame.")
          : ("No defined transform from parent " + parent_frame.get_name() +
              " frame to embedded " + child_frame.get_name() + " frame.")),
        parent_frame(parent_frame), child_frame(child_frame),
          is_child_to_parent(is_child_to_parent) {}
  }
}

#endif
