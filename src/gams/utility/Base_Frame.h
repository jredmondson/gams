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
#include <typeinfo>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>

#define INVAL_COORD DBL_MAX

namespace gams
{
  namespace utility
  {
    class Base_Frame;

    template<typename CoordType>
    class GAMS_Export Frame_Bound
    {
    public:
      static Base_Frame *default_frame;

      Frame_Bound() : frame(default_frame) {}

      Frame_Bound(const Base_Frame &frame) : frame(&frame) {}

      Frame_Bound(const Base_Frame *frame) : frame(frame) {}

      Frame_Bound(const Frame_Bound &orig) : frame(orig.frame) {}

    protected:
      const Base_Frame *frame;

    public:
      const Base_Frame &get_frame() const { return *frame; }
      CoordType transform_to(const Base_Frame &new_frame) const;
      double distance_to(const Frame_Bound<CoordType> &target) const;

      friend class Base_Frame;
    };

    class GAMS_Export Location_Base
    {
    public:
      Location_Base(double x, double y, double z = 0.0)
        : x(x), y(y), z(z) {}

      Location_Base()
        : x(INVAL_COORD), y(INVAL_COORD), z(INVAL_COORD) {}

      Location_Base(const Location_Base &orig)
        : x(orig.x), y(orig.y), z(orig.z) {}

      bool isInvalid() {
        return x == INVAL_COORD || y == INVAL_COORD || z == INVAL_COORD;
      }

      double x, y, z;
    };

    class GAMS_Export Location : public Location_Base, public Frame_Bound<Location>
    {
    public:
      Location(double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Frame_Bound() {}

      Location(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Frame_Bound(frame) {}

      Location() : Location_Base(), Frame_Bound() {}

      Location(const Location &orig)
        : Location_Base(orig), Frame_Bound(orig) {}

      typedef Location_Base Base_Type;
    };

    class GAMS_Export Rotation_Base
    {
    public:
      Rotation_Base(double alpha, double beta, double gamma)
        : alpha(alpha), beta(beta), gamma(gamma) {}

      Rotation_Base()
        : alpha(INVAL_COORD), beta(INVAL_COORD), gamma(INVAL_COORD) {}

      Rotation_Base(const Rotation_Base &orig)
        : alpha(orig.alpha), beta(orig.beta), gamma(orig.gamma) {}

      bool isInvalid() {
        return alpha == INVAL_COORD || beta == INVAL_COORD || gamma == INVAL_COORD;
      }

      double alpha, beta, gamma;
    };

    class GAMS_Export Rotation : public Rotation_Base, public Frame_Bound<Rotation>
    {
    public:
      Rotation(double alpha, double beta, double gamma)
        : Rotation_Base(alpha, beta, gamma), Frame_Bound() {}

      Rotation(const Base_Frame &frame, double alpha, double beta, double gamma)
        : Rotation_Base(alpha, beta, gamma), Frame_Bound(frame) {}

      Rotation() : Rotation_Base(), Frame_Bound() {}

      Rotation(const Rotation & orig)
        : Rotation_Base(orig), Frame_Bound(orig) {}

      typedef Rotation_Base Base_Type;
    };

    class GAMS_Export Pose_Base : public Location_Base, public Rotation_Base
    {
    public:
      Pose_Base(double x, double y, double z, double alpha, double beta, double gamma)
        : Location_Base(x, y, z), Rotation_Base(alpha, beta, gamma) {}

      Pose_Base(const Location_Base &loc)
        : Location_Base(loc), Rotation_Base(0, 0, 0) {}

      Pose_Base(const Location_Base &loc, const Rotation_Base &rot)
        : Location_Base(loc), Rotation_Base(rot) {}

      Pose_Base() : Location_Base(), Rotation_Base() {}

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }
    };

    class GAMS_Export Pose : public Pose_Base, public Frame_Bound<Pose>
    {
    public:
      Pose(double x, double y, double z, double alpha, double beta, double gamma)
        : Pose_Base(x, y, z, alpha, beta, gamma), Frame_Bound() {}

      Pose(double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Frame_Bound() {}

      Pose(const Base_Frame &frame, double x, double y, double z, double alpha, double beta, double gamma)
        : Pose_Base(x, y, z, alpha, beta, gamma), Frame_Bound(frame) {}

      Pose(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Frame_Bound(frame) {}

      Pose() : Pose_Base(), Frame_Bound() {}

      Pose(const Location &loc)
        : Pose_Base(loc), Frame_Bound(loc.get_frame()) {}

      Pose(const Base_Frame &frame, const Location_Base &loc, const Rotation_Base &rot)
        : Pose_Base(loc, rot), Frame_Bound(frame) {}

      /// Precondition: loc.frame == rot.frame
      Pose(const Location &loc, const Rotation &rot)
        : Pose_Base(loc, rot), Frame_Bound(loc.get_frame()) {}

      operator Location() const
      {
        return Location(*frame, x, y, z);
      }

      operator Rotation() const
      {
        return Rotation(*frame, alpha, beta, gamma);
      }

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }

      typedef Pose_Base Base_Type;
    };

    template <typename CoordType>
    class GAMS_Export Frame_Ops
    {
    protected:
      virtual CoordType transform_to_origin(const typename CoordType::Base_Type &in) const
      {
        throw std::runtime_error("Unsupported coordinate type for transform_to_origin");
      }

      virtual CoordType transform_from_origin(const typename CoordType::Base_Type &in) const
      {
        throw std::runtime_error("Unsupported coordinate type for transform_from_origin");
      }

      virtual double calc_distance(const typename CoordType::Base_Type &loc1, const typename CoordType::Base_Type &loc2) const
      {
        throw std::runtime_error("Unsupported coordinate type for calc_distance");
      }
    };

    class GAMS_Export Base_Frame
      : public Frame_Ops<Location>,
        public Frame_Ops<Rotation>
    {
    protected:
      using Frame_Ops<Location>::transform_to_origin;
      using Frame_Ops<Location>::transform_from_origin;
      using Frame_Ops<Location>::calc_distance;

      using Frame_Ops<Rotation>::transform_to_origin;
      using Frame_Ops<Rotation>::transform_from_origin;
      using Frame_Ops<Rotation>::calc_distance;

    public:
      Base_Frame() : origin(Pose(*this, 0, 0, 0, 0, 0, 0)) {}
      Base_Frame(const Pose &origin) : origin(origin) {}

      Pose origin;

      bool operator==(const Base_Frame &other) const
      {
        return this == &other;
      }

    private:
      // TODO implement O(height) algo instead of O(height ^ 2)
      static const Base_Frame *find_common_frame(const Base_Frame *from,
        const Base_Frame *to, std::vector<const Base_Frame *> *to_stack)
      {
        const Base_Frame *cur_to = to;
        for(;;)
        {
          const Base_Frame *cur_from = from;
          for(;;)
          {
            if(cur_to == cur_from)
            {
              return cur_to;
            }
            const Base_Frame *next_cur_from = cur_from->origin.frame;
            if(cur_from == next_cur_from)
              break;
            cur_from = next_cur_from;
          }
          if(to_stack)
            to_stack->push_back(cur_to);
          const Base_Frame *next_cur_to = cur_to->origin.frame;
          if(cur_to == next_cur_to)
            break;
          cur_to = next_cur_to;
        }
        return NULL;
      }

    public:
      template<typename CoordType>
      static CoordType transform(const CoordType &in, const Base_Frame &to_frame)
      {
        if (to_frame == *in.frame)
          return in;
        else if (to_frame == *in.frame->origin.frame)
          return in.frame->transform_to_origin(in);
        else if (*to_frame.origin.frame == *in.frame)
          return to_frame.transform_from_origin(in);
        else
        {
          std::vector<const Base_Frame *> to_stack;
          const Base_Frame *transform_via = find_common_frame(in.frame, &to_frame, &to_stack);
          if(transform_via == NULL)
            throw std::runtime_error("No transform path found");
          else
          {
            CoordType ret = in;
            const Base_Frame *cur_frame = in.frame;
            while(cur_frame != transform_via)
            {
              ret = cur_frame->transform_to_origin(ret);
              cur_frame = ret.frame;
            }
            while(cur_frame != &to_frame && !to_stack.empty())
            {
              ret = to_stack.back()->transform_from_origin(ret);
              to_stack.pop_back();
              cur_frame = ret.frame;
            }
            return ret;
          }
        }
      }

      template<typename CoordType>
      static double distance(const CoordType &coord1, const CoordType &coord2)
      {
        if (&coord1.frame == &coord2.frame)
        {
          return calc_distance_within_frame(coord1, coord2, *coord1.frame);
        }
        else
        {
          CoordType coord2_conv = transform(coord2, *coord1.frame);
          return calc_distance_within_frame(coord1, coord2_conv, *coord1.frame);
        }
      }
    protected:
      template<typename CoordType>
      static double calc_distance_within_frame(const CoordType &coord1,
                        const CoordType &coord2, const Base_Frame &frame)
      {
        return frame.calc_distance(coord1, coord2);
      }

      virtual Pose transform_to_origin(const Pose_Base &in) const
      {
        return Pose(transform_to_origin(static_cast<const Location_Base &>(in)),
                    transform_to_origin(static_cast<const Rotation_Base &>(in)));
      }

      virtual Pose transform_from_origin(const Pose_Base &in) const
      {
        return Pose(transform_from_origin(static_cast<const Location_Base &>(in)),
                    transform_from_origin(static_cast<const Rotation_Base &>(in)));
      }
    };

    template<>
    inline double Base_Frame::calc_distance_within_frame<>(const Pose &pose1,
                      const Pose &pose2, const Base_Frame &frame)
    {
      return frame.calc_distance(static_cast<const Location_Base&>(pose1),
                                 static_cast<const Location_Base&>(pose2));
    }

    template<typename CoordType>
    inline CoordType Frame_Bound<CoordType>::transform_to(const Base_Frame &new_frame) const
    {
      return Base_Frame::transform(static_cast<const CoordType &>(*this), new_frame);
    }

    template<typename CoordType>
    inline double Frame_Bound<CoordType>::distance_to(const Frame_Bound<CoordType> &target) const
    {
      return Base_Frame::distance(static_cast<const CoordType &>(*this),
                                  static_cast<const CoordType &>(target));
    }

    /**
     * Locations represented as meters distance, in x, y, and z, from an origin point
     * Rotations represented as Euler angles, in the origin frame, applied in this order:
     *    alpha degrees around X-axis
     *    beta  degrees around Y-axis
     *    gamma degrees around Z-axis
     **/
    class GAMS_Export Cartesian_Frame : public Base_Frame
    {
    public:
      Cartesian_Frame() : Base_Frame() {}
      Cartesian_Frame(const Pose &origin) : Base_Frame(origin) {}

    protected:
      virtual Location transform_to_origin(const Location_Base &in) const
      {
        const std::type_info &origin_type(typeid(origin.get_frame()));
        if(origin_type == typeid(Cartesian_Frame))
        {
          std::cerr << "Conversion to origin: " << in.x << " to " << in.x + origin.x << std::endl;
          return Location(origin.get_frame(), in.x + origin.x, in.y + origin.y, in.z + origin.z);
        }
        else
        {
          throw std::runtime_error("Unsupported conversion to origin");
        }
      }

      virtual Location transform_from_origin(const Location_Base &in) const
      {
        const std::type_info &origin_type(typeid(origin.get_frame()));
        if(origin_type == typeid(Cartesian_Frame))
        {
          std::cerr << "Conversion from origin: " << in.x << " to " << in.x - origin.x << std::endl;
          return Location(*this, in.x - origin.x, in.y - origin.y, in.z - origin.z);
        }
        else
        {
          throw std::runtime_error("Unsupported conversion from origin");
        }
      }

      virtual double calc_distance(const Location_Base &loc1, const Location_Base &loc2) const
      {
        double x_dist = loc2.x - loc1.x;
        double y_dist = loc2.y - loc1.y;
        double z_dist = loc2.z - loc1.z;

        return sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);
      }

      virtual Rotation transform_to_origin(const Rotation_Base &in) const
      {
        return Rotation(origin.get_frame(), in.alpha, in.beta, in.gamma);
      }

      virtual Rotation transform_from_origin(const Rotation_Base &in) const
      {
        return Rotation(*this, in.alpha, in.beta, in.gamma);
      }

      virtual double calc_distance(const Rotation_Base &loc1, const Rotation_Base &loc2) const
      {
        return 0;
      }
    };

    /**
     * Locations represented as GPS coordinates, and an altitude,
     * assuming a spherical Earth:
     *    x is Latitude
     *    y is Longitude
     *    z is Altitude
     * Rotations represented as Euler angles, applied in this order:
     *    alpha degrees around axis pointing towards north pole at current location
     *    beta degrees around axis pointing west at current location
     *    gamma degrees around axis pointing upwards (i.e, normal vector) at current location
     *
     *    Note that rotations other than gamma are undefined at north and south poles.
     **/
    class GAMS_Export GPS_Frame : public Base_Frame
    {
    };
  }
}

#endif
