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

#ifndef _GAMS_UTILITY_COORDINATE_H_
#define _GAMS_UTILITY_COORDINATE_H_

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

#define INVAL_COORD DBL_MAX

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    /**
     * For internal use.
     *
     * Workaround to allow all specializations of Coordinate to share the
     * same default frame;
     **/
    class Coordinate_Base
    {
    protected:
      GAMS_Export static const Reference_Frame *default_frame;
    };

    /**
     * New coordinate types which are frame-dependant can inherit from this
     * class. Pass the type of the child class as CoordType
     *
     * New coordinate types must:
     *   -- Inherit from Coordinate, and pass itself as template parameter
     *   -- Inherit from a base class which does not inherit Coordinate
     *   -- That base class must:
     *      -- Have a typedef Base_Type which refers to itself
     *      -- Have this function: static std::string name()
     **/
    template<typename CoordType>
    class Coordinate : private Coordinate_Base
    {
    public:

      Coordinate() : _frame(default_frame) {}

      explicit Coordinate(const Reference_Frame &frame) : _frame(&frame) {}

      explicit Coordinate(const Reference_Frame *frame) : _frame(frame) {}

      Coordinate(const Coordinate &orig) : _frame(&orig.frame()) {}

    private:
      const Reference_Frame *_frame;

      CoordType &as_coord_type()
      {
        return static_cast<CoordType &>(*this);
      }

      const CoordType &as_coord_type() const
      {
        return static_cast<const CoordType &>(*this);
      }

      template<typename Type>
      Type &as_type()
      {
          return static_cast<Type &>(as_coord_type());
      }

      template<typename Type>
      const Type &as_type() const
      {
          return static_cast<const Type &>(as_coord_type());
      }

    public:
      const Reference_Frame &frame() const { return *_frame; }
      const Reference_Frame &frame(const Reference_Frame &new_frame) {
        return *(_frame = &new_frame);
      }

      // Defined in Reference_Frame.h
      CoordType transform_to(const Reference_Frame &new_frame) const;
      void transform_this_to(const Reference_Frame &new_frame);
      double distance_to(const CoordType &target) const;
      void normalize();

      bool operator==(const CoordType &other) const
      {
        if(frame() == other.frame())
        {
          return as_type<typename CoordType::Base_Type>() == other;
        }
        else
        {
          CoordType tmp(frame(), static_cast<const CoordType &>(other));
          return as_type<typename CoordType::Base_Type>() == tmp;
        }
      }

      bool operator!=(const CoordType &other) const
      {
        return !(*this == other);
      }

      bool approximately_equal(const CoordType &other, double epsilon) const
      {
        return distance_to(other) <= epsilon;
      }

      /**
       * Less than used for ordering in stl containers
       * @param rhs   comparing position
       * @return true if *this is less than rhs
       **/
      bool operator<(const Coordinate<CoordType> &rhs) const
      {
        const CoordType &s = as_coord_type();
        const CoordType &o = rhs.as_coord_type();

        for(int i = 0; i < s.size(); ++i)
        {
          double l = s.get(i);
          double r = o.get(i);
          if(l<r)
            return true;
          if(r<l)
            return false;
        }
        return false;
      }

      std::string to_string(const std::string &delim = ",") const
      {
        std::stringstream buffer;
        const CoordType &s = as_coord_type();
        for(int i = 0; i < s.size(); ++i)
        {
          if(i > 0)
            buffer << delim;
          buffer << s.get(i);
        }
        return buffer.str();
      }

    private:
      static std::istream &skip_nonnum(std::istream &s)
      {
        int next;
        while(s && (next = s.peek()) != EOF)
        {
          if(next == '.' || next == '-' || (next >= '0' && next <= '9'))
            break;
          s.get();
        }
        return s;
      }
    public:

      /**
       * Sets coordinates from a string encoding a sequence of doubles,
       * separated by any set of characters other than 0-9, '.', and '-'
       **/
      void from_string(const std::string &in)
      {
        std::stringstream buffer(in);
        CoordType &s = as_coord_type();
        for(int i = 0; i < s.size(); ++i)
        {
          double val;
          buffer >> skip_nonnum;
          buffer >> val;
          s.set(i, val);
        }
      }

      template<typename ContainType>
      void to_container(ContainType &container) const
      {
        const CoordType &s = as_coord_type();
        for(int i = 0; i < s.size(); i++)
        {
          container.set(i, s.get(i));
        }
      }

      template<typename ContainType>
      void from_container(ContainType &container)
      {
        CoordType &s = as_coord_type();
        for(int i = 0; i < s.size(); i++)
        {
          s.set(i, container[i]);
        }
      }
    };
  }
}

// Include if not already included
#include <gams/utility/Reference_Frame.h>

#endif
