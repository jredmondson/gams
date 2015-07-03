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
 * @file Reference_Frame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_REFERENCE_FRAME_H_
#define _GAMS_UTILITY_REFERENCE_FRAME_H_

#include "gams/GAMS_Export.h"
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>
#include <gams/utility/Pose.h>

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
            dynamic_cast<const type *>(&coord.frame()), \
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
      bad_coord_type(const Reference_Frame &frame, const std::string &fn_name);
      ~bad_coord_type() throw() {}

      std::string coord_type_name;
      std::string fn_name;
      const Reference_Frame &frame;

      typedef CoordType Coord_Type;
    };

    /**
     * Thrown when an an attempt is made to transform between frames
     * that do not belong to the same frame tree.
     **/
    class unrelated_frames : public std::runtime_error
    {
    public:
      unrelated_frames(const Reference_Frame &from_frame,
        const Reference_Frame &to_frame) :
        std::runtime_error("No transform path found between frames."),
        from_frame(from_frame), to_frame(to_frame) {}
      ~unrelated_frames() throw() {}

      const Reference_Frame &from_frame;
      const Reference_Frame &to_frame;
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
      undefined_transform(const Reference_Frame &parent_frame,
        const Reference_Frame &child_frame, bool is_child_to_parent,
        bool unsupported_rotation = false);
      ~undefined_transform() throw() {}

      const Reference_Frame &parent_frame;
      const Reference_Frame &child_frame;
      bool is_child_to_parent;
      bool unsupported_rotation;
    };

    /**
     * Base class for Reference Frames.
     * Inherit from this class if implementing a new reference frame.
     * Otherwise, do not use directly.
     **/
    class GAMS_Export Reference_Frame
    {
    public:
      /**
       * Default constructor. No parent frame.
       **/
      Reference_Frame()
        : _origin(new Pose(*this, 0, 0, 0, 0, 0, 0)),
          _destruct_origin(true) {}

      /**
       * Creates a copy of the origin Pose passed in.
       **/
      explicit Reference_Frame(const Pose &origin)
        : _origin(new Pose(origin)),
          _destruct_origin(true) {}

      /**
       * Uses an existing Pose as origin, and maintains
       * a pointer to it. Changes to it affect this frame
       **/
      explicit Reference_Frame(Pose *origin)
        : _origin(origin),
          _destruct_origin(false) {}

      virtual ~Reference_Frame()
      {
        if(_destruct_origin)
        {
          delete _origin;
        }
      }

      const Pose &origin() const { return *_origin; }

      const Pose &origin(const Pose &new_origin)
      {
        if(_destruct_origin)
          delete _origin;
        _origin = NULL;
        _destruct_origin = true;
        return *(_origin = new Pose(new_origin));
      }

      const Reference_Frame &origin_frame() const { return origin().frame(); }

      /**
       * Bind this frame's origin to a Pose. Changes to that
       * Pose will affect this frame.
       **/
      void bind_origin(Pose *new_origin)
      {
        if(_destruct_origin)
          delete _origin;
        _origin = NULL;
        _destruct_origin = false;
        _origin = new_origin;
      }

      bool operator==(const Reference_Frame &other) const
      {
        return this == &other;
      }

      bool operator!=(const Reference_Frame &other) const
      {
        return !(*this == other);
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

      /**
       * Returns a human-readable name for the coordinate system type
       **/
      std::string name() const { return get_name(); };

    private:
      /**
       * Override to return a human-readable name for new coordinate systems
       *
       * @return the name
       **/
      virtual std::string get_name() const = 0;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param in transforms parameter into origin frame from this frame.
       **/
      virtual void transform_to_origin(Location_Vector &in) const
      {
        throw bad_coord_type<Location>(*this, "transform_to_origin");
      }

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param in transforms parameter into this frame from origin frame.
       **/
      virtual void transform_from_origin(Location_Vector &in) const
      {
        throw bad_coord_type<Location>(*this, "transform_from_origin");
      }

      /**
       * Override for new coordinate systems that can require normalization of
       * coordinates, e.g., angle-based systems. NOP by default.
       *
       * @param in transforms parameter into normalized form
       **/
      virtual void do_normalize(Location_Vector &in) const
      {
      }

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param loc1 Distance from this location
       * @param loc2 Distance to this location
       * @return distance in meters from loc1 to loc2
       **/
      virtual double calc_distance(const Location_Vector &loc1, const Location_Vector &loc2) const
      {
        throw bad_coord_type<Location>(*this, "calc_distance");
      }

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param in transforms parameter into origin frame from this frame.
       **/
      virtual void transform_to_origin(Rotation_Vector &in) const
      {
        throw bad_coord_type<Rotation>(*this, "transform_to_origin");
      }

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param in transforms parameter into this frame from origin frame.
       **/
      virtual void transform_from_origin(Rotation_Vector &in) const
      {
        throw bad_coord_type<Rotation>(*this, "transform_from_origin");
      }

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param rot1 Distance from this rotation
       * @param rot2 Distance to this rotation
       * @return rotatioal distance in degrees from rot1 to rot2
       **/
      virtual double calc_distance(const Rotation_Vector &rot1, const Rotation_Vector &rot2) const
      {
        throw bad_coord_type<Rotation>(*this, "calc_distance");
      }

      /**
       * Override for new coordinate systems that can require normalization of
       * coordinates, e.g., angl-based systems. NOP by default.
       *
       * @param in transforms parameter into normalized form
       **/
      virtual void do_normalize(Rotation_Vector &in) const
      {
      }

      /**
       * Transform input coordinates into their common parent. If no common
       * parent exists, throws, unrelated_frames exception
       *
       * Returns the common frame the coordinates were transformed to.
       **/
      template<typename CoordType>
      static const Reference_Frame &common_parent_transform(CoordType &in1, CoordType &in2)
      {
        if(in1.frame() == in2.frame())
          return in1.frame();
        else if (in1.frame() == in2.frame().origin_frame())
        {
          transform_to_origin_within_frame(in2, in2.frame());
          return in2.frame(in1.frame());
        }
        else if (in2.frame() == in1.frame().origin_frame())
        {
          transform_to_origin_within_frame(in1, in1.frame());
          return in1.frame(in2.frame());
        }
        else
          return common_parent_transform_other(in1, in2);
      }

    private:
      Pose *_origin;
      bool _destruct_origin;

      template<typename CoordType>
      static const Reference_Frame &common_parent_transform_other(CoordType &in1, CoordType &in2);

      /**
       * Transform coordinate `in` from its current from, to the specified frame
       **/
      template<typename CoordType>
      static void transform(CoordType &in, const Reference_Frame &to_frame)
      {
        if (to_frame == in.frame())
          return;
        else if (to_frame == in.frame().origin_frame())
        {
          transform_to_origin_within_frame(in, in.frame());
          in.frame(in.frame().origin_frame());
        }
        else if (to_frame.origin_frame() == in.frame())
        {
          transform_from_origin_within_frame(in, to_frame);
          in.frame(to_frame);
        }
        else
          transform_other(in, to_frame);
      }

      /**
       * Calculate distances between coordinates. If they have different
       * frames, first transform to their lowest common parent
       **/
      template<typename CoordType>
      static double distance(const CoordType &coord1, const CoordType &coord2)
      {
        if (coord1.frame() == coord2.frame())
        {
          return calc_distance_within_frame(coord1, coord2, coord1.frame());
        }
        else
        {
          CoordType coord1_conv(coord1);
          CoordType coord2_conv(coord2);
          const Reference_Frame &pframe = common_parent_transform(coord1_conv, coord2_conv);
          return calc_distance_within_frame(coord1_conv, coord2_conv, pframe);
        }
      }

      /**
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       **/
      template<typename CoordType>
      static void transform_to_origin_within_frame(CoordType &in,
          const Reference_Frame &frame)
      {
        frame.transform_to_origin(in);
      }

      /**
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       **/
      template<typename CoordType>
      static void transform_from_origin_within_frame(CoordType &in,
          const Reference_Frame &frame)
      {
        frame.transform_from_origin(in);
      }

      /**
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       **/
      template<typename CoordType>
      static double calc_distance_within_frame( const CoordType &coord1,
          const CoordType &coord2, const Reference_Frame &frame)
      {
        return frame.calc_distance(coord1, coord2);
      }

      static const Reference_Frame *find_common_frame(const Reference_Frame *from,
        const Reference_Frame *to, std::vector<const Reference_Frame *> *to_stack = NULL);

      template<typename CoordType>
      inline static void transform_other(CoordType &in, const Reference_Frame &to_frame);

      template<typename CoordType>
      friend class Coordinate;
    };

    /**
     * For internal use.
     *
     * Provides implementation of transforms between rotations represented using
     * 3-value axis/angle notation. Inherit from this to use this implementation.
     **/
    class GAMS_Export Axis_Angle_Frame : public Reference_Frame
    {
    protected:
      Axis_Angle_Frame() : Reference_Frame() {}

      explicit Axis_Angle_Frame(const Pose &origin)
        : Reference_Frame(origin) {}

      explicit Axis_Angle_Frame(Pose *origin)
        : Reference_Frame(origin) {}

    protected:
      void rotate_location_vec(Location_Vector &loc, const Rotation_Vector &rot, bool reverse = false) const;

    private:
      virtual void transform_to_origin(Rotation_Vector &in) const;

      virtual void transform_from_origin(Rotation_Vector &in) const;

      virtual double calc_distance(const Rotation_Vector &loc1, const Rotation_Vector &loc2) const
      {
        Quaternion quat1(loc1);
        Quaternion quat2(loc2);
        return RAD_TO_DEG(quat1.angle_to(quat2));
      }
    };

    template<>
    inline double Reference_Frame::calc_distance_within_frame<>(const Pose &pose1,
                      const Pose &pose2, const Reference_Frame &frame)
    {
      return frame.calc_distance(static_cast<const Location_Vector&>(pose1),
                                 static_cast<const Location_Vector&>(pose2));
    }

    template<>
    inline void Reference_Frame::transform_to_origin_within_frame<>(
      Pose &in, const Reference_Frame &frame)
    {
      frame.transform_to_origin(static_cast<Location_Vector &>(in));
      frame.transform_to_origin(static_cast<Rotation_Vector &>(in));
    }

    template<>
    inline void Reference_Frame::transform_from_origin_within_frame<>(
      Pose &in, const Reference_Frame &frame)
    {
      frame.transform_from_origin(static_cast<Location_Vector &>(in));
      frame.transform_from_origin(static_cast<Rotation_Vector &>(in));
    }

    template<>
    inline void Reference_Frame::normalize<>(Pose &pose) const
    {
      pose.frame().do_normalize(static_cast<Location_Vector&>(pose));
      pose.frame().do_normalize(static_cast<Rotation_Vector&>(pose));
    }

    template<typename CoordType>
    inline CoordType Coordinate<CoordType>::transform_to(const Reference_Frame &new_frame) const
    {
      CoordType ret(static_cast<const CoordType &>(*this));
      Reference_Frame::transform(ret, new_frame);
      return ret;
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::transform_this_to(const Reference_Frame &new_frame)
    {
      Reference_Frame::transform(static_cast<CoordType &>(*this), new_frame);
    }

    template<typename CoordType>
    inline double Coordinate<CoordType>::distance_to(const CoordType &target) const
    {
      return Reference_Frame::distance(static_cast<const CoordType &>(*this),
                                       static_cast<const CoordType &>(target));
    }

    template<typename CoordType>
    inline void Coordinate<CoordType>::normalize()
    {
      frame().normalize(static_cast<CoordType &>(*this));
    }

    template<typename CoordType>
    inline void Reference_Frame::transform_other(CoordType &in, const Reference_Frame &to_frame)
    {
      std::vector<const Reference_Frame *> to_stack;
      const Reference_Frame *transform_via = find_common_frame(&in.frame(), &to_frame, &to_stack);
      if(transform_via == NULL)
        throw unrelated_frames(in.frame(), to_frame);
      else
      {
        while(in.frame() != *transform_via)
        {
          transform_to_origin_within_frame(in, in.frame());
          in.frame(in.frame().origin_frame());
        }
        while(in.frame() != to_frame && !to_stack.empty())
        {
          transform_from_origin_within_frame(in, *to_stack.back());
          in.frame(*to_stack.back());
          to_stack.pop_back();
        }
      }
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

    template<typename CoordType>
    inline const Reference_Frame &Reference_Frame::common_parent_transform_other(CoordType &in1, CoordType &in2)
    {
      const Reference_Frame *common_parent = find_common_frame(&in1.frame(), &in2.frame());
      if(common_parent == NULL)
        throw unrelated_frames(in1.frame(), in2.frame());
      else
      {
        std::cout << common_parent->origin() << std::endl;
        while(in1.frame() != *common_parent)
        {
          transform_to_origin_within_frame(in1, in1.frame());
          in1.frame(in1.frame().origin_frame());
        }
        while(in2.frame() != *common_parent)
        {
          transform_to_origin_within_frame(in2, in2.frame());
          in2.frame(in2.frame().origin_frame());
        }
      }
      return *common_parent;
    }

    template<typename CoordType>
    inline bad_coord_type<CoordType>
          ::bad_coord_type(const Reference_Frame &frame, const std::string &fn_name)
      : std::runtime_error("Coordinate type " + CoordType::name() +
            " not supported by frame type " + frame.name() +
            " for operation " + fn_name + "."),
        coord_type_name(CoordType::name()),
        fn_name(fn_name), frame(frame) {}

    inline undefined_transform::undefined_transform(
            const Reference_Frame &parent_frame, const Reference_Frame &child_frame,
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
  }
}

#endif
