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
 * @file Coordinate.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location, Rotation, and Pose classes
 **/

#ifndef _GAMS_UTILITY_COORDINATE_H_
#define _GAMS_UTILITY_COORDINATE_H_

#include "gams/GAMSExport.h"
#include <gams/CPP11_compat.h>
#include <string>
#include <cfloat>

#define INVAL_COORD DBL_MAX

namespace gams
{
  namespace utility
  {
    class ReferenceFrame;

    /**
     * For internal use.
     *
     * Allows all specializations of Coordinate to share the same default frame.
     * This type serves no other purpose.
     **/
    class CoordinateBase
    {
    public:
      /**
       * Retrieves the default frame that Coordinates (Pose, Location, Rotation)
       * that don't specify a frame will use.
       *
       * @return a reference to a CartesianFrame object that serves as default
       **/
      GAMSExport static const ReferenceFrame &default_frame();

    private:
      GAMSExport static const ReferenceFrame *default_frame_;
    };

    /**
     * New coordinate types which are frame-dependant can inherit from this
     * class. Pass the type of the child class as CoordType
     *
     * New coordinate types must:
     *   -- Inherit from Coordinate, and pass itself as template parameter
     *   -- Inherit from a base class which does not inherit from Coordinate
     *   -- That base class must:
     *      -- Have a typedef BaseType which refers to itself
     *      -- Implement operator==
     *      -- Implement the following methods:
     *   static std::string name() // return the type's name
     *   static constexpr int size() // return # of values in representation
     *   constexpr double get(int i) const // return ith value of representation
     *   constexpr bool operator==(const BaseType &rhs) const
     *   BaseType &as_vec() // return *this
     *   constexpr const BaseType &as_vec() const // return *this
     *
     * Additionally, new coordinate types should either:
     *   -- Specialize the ReferenceFrame::*_within_frame templates; OR
     *   -- Add new overloads for transform_to_origin, transform_from_origin,
     *      do_normalize, and calc_distance virtual methods in ReferenceFrame,
     *      and add transformation logic for those methods in the various frames
     **/
    template<typename CoordType>
    class Coordinate : public CoordinateBase
    {
    public:
      /**
       * Default Constructor. Initializes frame as default_frame()
       **/
      Coordinate();

      /**
       * Construct through a reference to a frame object. This Coordinate must
       * not outlive the ReferenceFrame that is passed in.
       *
       * @param frame the reference frame this Coordinate will belong to
       **/
      constexpr explicit Coordinate(const ReferenceFrame &frame);

      /**
       * Construct through a pointer to a frame object. This Coordinate must not
       * outlive the ReferenceFrame that is passed in.
       *
       * @param frame the reference frame this Coordinate will belong to
       **/
      constexpr explicit Coordinate(const ReferenceFrame *frame);

      /**
       * Copy constructor. This Coordinate will refer to the same frame as the
       * original Coordinate.
       *
       * @param orig the original Coordinate to copy.
       **/
      constexpr Coordinate(const Coordinate &orig);

      /**
       * Getter for the reference frame this Coordinate belongs to.
       *
       * @return the frame
       **/
      constexpr const ReferenceFrame &frame() const;

      /**
       * Setter for the reference frame this Coordinate belongs to. Any further
       * calculations using this Coordinate will use this frame.
       *
       * Not thread-safe.
       *
       * @param new_frame the frame the Coordinate will now belong to
       * @return the new frame
       **/
      const ReferenceFrame &frame(const ReferenceFrame &new_frame);

      /**
       * Evaluate equality with the other Coordinate. If the coordinates belong
       * to different reference frames, the rhs will be copied, and converted
       * to the same frame as the lhs
       *
       * Note: in practice, the approximately_equal method may be more useful
       *
       * @param rhs the other Coordinate
       * @return true if, after conversion, the two coordinates are the same
       **/
      bool operator==(const CoordType &rhs) const;

      /**
       * Evaluate equality with a vector-like object
       *
       * @tparam RhsType type of the right-hand-side. Must support a size()
       *     method, and operator[] to access contents.
       * @param rhs the right-hand-side of the equality. This object is treated
       *     as a set of coordinates in this Coordinate's reference frame.
       * @return true if the rhs is the same size as this object, and all terms
       *     are equal to each other, respectively. False otherwise.
       **/
      template<typename RhsType>
      bool operator==(const RhsType &rhs) const;

      /**
       * Evaluate inequality with the other Coordinate. If the coordinates
       * belong to different reference frames, the rhs will be copied, and
       * converted to the same frame as the lhs
       *
       * Note: in practice, a negated call to the approximately_equal method
       * may be more useful
       *
       * @param rhs the other Coordinate
       * @return false if, after conversion, the two coordinates are the same
       **/
      bool operator!=(const CoordType &rhs) const;

      /**
       * Evaluate inequality with a vector-like object
       *
       * @tparam RhsType type of the right-hand-side. Must support a size()
       *     method, and operator[] to access contents.
       * @param rhs the right-hand-side of the equality. This object is treated
       *     as a set of coordinates in this Coordinate's reference frame.
       * @return false if the rhs is the same size as this object, and all terms
       *     are equal to each other, respectively. True otherwise.
       **/
      template<typename RhsType>
      bool operator!=(const RhsType &rhs) const;

      /**
       * Tests if this Coordinate is within epsilon in distance (as defined by
       * this Coordinate's reference frame's distance metric). If the other
       * Coordinate is in a different reference frame, it is first copied, and
       * converted to this Coordinate's reference frame.
       *
       * @param other the other Coordinate to test against
       * @param epsilon the maximum distance permitted to return true
       * @return true if the distance is less than or equal to  epsilon
       **/
      bool approximately_equal(const CoordType &other, double epsilon) const;

      /**
       * Less than used for ordering in stl containers
       * @param rhs   comparing position
       * @return true if *this is less than rhs
       **/
      bool operator<(const Coordinate<CoordType> &rhs) const;

      /**
       * Converts this Coordinate to a string; each value in the data
       * tuple is joined with the delimeter passed in.
       *
       * @param delim the delimiter to join the values with; defaults to a comma
       * @return the string representation of this Coordinate. If fed into the
       *    from_string method of the same Coordinate type within the same
       *    referenence frame, will produce an equal Coordinate to this one.
       **/
      std::string to_string(const std::string &delim = ",") const;

      /**
       * Sets coordinates from a string encoding a sequence of doubles,
       * separated by any set of characters other than 0-9, '.', and '-'
       *
       * This does not modify this Coordinate's reference frame binding.
       * The new coordinate will be within this Coordinate's reference frame.
       *
       * @param in  the input string, as specified above
       **/
      void from_string(const std::string &in);

      /**
       * Outputs this Coordinates values to the referenced container. This
       * container type must support a set method taking two parameters:
       *  first, an integer index; second, a floating-point value
       *
       * The MADARA DoubleVector and NativeDoubleVector types are supported.
       *
       * @tparam ContainType the type of the container; must support "set"
       * @param container the container to put this Coordinate's values into.
       **/
      template<typename ContainType>
      void to_container(ContainType &container) const;

      /**
       * Overwrites this Coordinate's values with those pulled from the
       * referenced container. These values will be within this object's
       * current reference frame. The container must support operator[],
       *
       * @tparam ContainType the type of the container; must support operator[]
       * @param container the container to pull new values from.
       **/
      template<typename ContainType>
      void from_container(const ContainType &container);


      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       * The four methods below are defined in ReferenceFrame.inl,
       * due to circular dependencies
       * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

      /**
       * Copy and transform this coordinate to a new reference frame
       *
       * @param new_frame the frame to transform to
       * @return the new coordinate in the new frame
       *
       * @throws bad_coord_type thrown if the new reference frame does
       *      not support CoordType.
       * @throws unrelated_frames thrown if the new reference frame is not
       *      part of the same tree as the current one.
       * @throws undefined_transform thrown if no conversion between two frames
       *      along the conversion path has been defined.
       **/
      CoordType transform_to(const ReferenceFrame &new_frame) const;

      /**
       * Transform this coordinate, in place, to a new reference frame
       *
       * @param new_frame the frame to transform to
       *
       * @throws bad_coord_type thrown if the new reference frame does
       *      not support CoordType.
       * @throws unrelated_frames thrown if the new reference frame is not
       *      part of the same tree as the current one.
       * @throws undefined_transform thrown if no conversion between two frames
       *      along the conversion path has been defined.
       **/
      void transform_this_to(const ReferenceFrame &new_frame);

      /**
       * Calculate distance from this Coordinate to a target. If the target
       * is in another reference frame, this and the target will be copied, and
       * converted to their closest common frame.
       *
       * @param target the target Coordinate to calculate distance to
       * @return the distance according to the distance metric in the common
       *   frame, for CoordType. Typically, return will be meters or degrees.
       *
       * @throws bad_coord_type thrown if a frame along the conversion path
       *      does not support CoordType.
       * @throws unrelated_frames thrown if the target's reference frame is not
       *      part of the same tree as the current one.
       * @throws undefined_transform thrown if no conversion between two frames
       *      along the conversion path has been defined.
       **/
      double distance_to(const CoordType &target) const;

      /**
       * Reduces this Coordinate to it's normalized form, should one exist.
       * Typically useful for Coordinate types which incorporate angles.
       **/
      void normalize();

    private:
      const ReferenceFrame *frame_;

      CoordType &as_coord_type();

      constexpr const CoordType &as_coord_type() const;

      template<typename Type>
      Type &as_type();

      template<typename Type>
      constexpr const Type &as_type() const;
    };
  }
}

#include "Coordinate.inl"

// Include if not already included
#include <gams/utility/Pose.h>

#endif
