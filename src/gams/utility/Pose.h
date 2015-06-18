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
 * @file Coordinates.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location, Rotation, and Pose classes
 **/

#ifndef _GAMS_UTILITY_POSE_H_
#define _GAMS_UTILITY_POSE_H_

#include "gams/GAMS_Export.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>
#include <cstdio>

#include <gams/utility/Location.h>
#include <gams/utility/Rotation.h>

namespace gams
{
  namespace utility
  {
    class Base_Frame;

    /**
     * Container for Pose information, not bound to a frame.
     * Do not use unless implementing new frames or coordinate types.
     **/
    class GAMS_Export Pose_Base : public Location_Base, public Rotation_Base
    {
    public:
      Pose_Base(double x, double y, double z, double rx, double ry, double rz)
        : Location_Base(x, y, z), Rotation_Base(rx, ry, rz) {}

      Pose_Base(const Location_Base &loc)
        : Location_Base(loc), Rotation_Base(0, 0, 0) {}

      Pose_Base(const Rotation_Base &rot)
        : Location_Base(0, 0, 0), Rotation_Base(rot) {}

      Pose_Base(const Location_Base &loc, const Rotation_Base &rot)
        : Location_Base(loc), Rotation_Base(rot) {}

      Pose_Base() : Location_Base(), Rotation_Base() {}

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }

      bool operator==(const Pose_Base &other) const
      {
        return as_location() == other.as_location() &&
               as_rotation() == other.as_rotation();
      }

      static std::string name()
      {
        return "Pose";
      }

      int cardinality() const
      {
        return 6;
      }

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double get(int i) const
      {
        if(i <= 2)
          return as_location().get(i);
        else
          return as_rotation().get(i - 3);
      }

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double set(int i, double val)
      {
        if(i <= 2)
          return as_location().set(i, val);
        else
          return as_rotation().set(i - 3, val);
      }

      typedef Pose_Base Base_Type;
    private:
      Location_Base &as_location()
      {
        return static_cast<Location_Base &>(*this);
      }

      const Location_Base &as_location() const
      {
        return static_cast<const Location_Base &>(*this);
      }

      Rotation_Base &as_rotation()
      {
        return static_cast<Rotation_Base &>(*this);
      }

      const Rotation_Base &as_rotation() const
      {
        return static_cast<const Rotation_Base &>(*this);
      }
    };

    /**
     * Represents a combination of Location and Rotation within a single
     * reference frame.
     **/
    class GAMS_Export Pose : public Pose_Base, public Coordinate<Pose>
    {
    public:
      Pose(double x, double y, double z, double rx, double ry, double rz)
        : Pose_Base(x, y, z, rx, ry, rz), Coordinate() {}

      Pose(double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Coordinate() {}

      Pose(const Base_Frame &frame, double x, double y, double z, double rx, double ry, double rz)
        : Pose_Base(x, y, z, rx, ry, rz), Coordinate(frame) {}

      Pose(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Coordinate(frame) {}

      Pose() : Pose_Base(), Coordinate() {}

      Pose(const Location &loc)
        : Pose_Base(loc), Coordinate(loc.frame()) {}

      Pose(const Rotation &rot)
        : Pose_Base(rot), Coordinate(rot.frame()) {}

      Pose(const Base_Frame &frame, const Location_Base &loc, const Rotation_Base &rot)
        : Pose_Base(loc, rot), Coordinate(frame) {}

      /// Precondition: loc.frame == rot.frame
      Pose(const Location &loc, const Rotation &rot)
        : Pose_Base(loc, rot), Coordinate(loc.frame()) {}

      operator Location() const
      {
        return Location(frame(), x(), y(), z());
      }

      operator Rotation() const
      {
        return Rotation(frame(), rx(), ry(), rz());
      }

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }

      using Coordinate<Pose>::operator==;
    };
  }
}

// Include if not already included
#include <gams/utility/Base_Frame.h>

#endif
