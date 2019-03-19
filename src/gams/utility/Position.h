/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Position.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a utility class for working with position
 *
 * DEPRECATED: use pose::Position instead. Will be removed in GAMS v2
 **/

#ifndef _GAMS_UTILITY_POSITION_H_
#define _GAMS_UTILITY_POSITION_H_

#include <vector>

#include "gams/GamsExport.h"
#include "madara/knowledge/containers/DoubleVector.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"

#include "gams/pose/Position.h"
#include "gams/pose/ReferenceFrame.h"

namespace gams
{
  namespace utility
  {
    /**
    * A position in an x, y, z coordinate system
    *
    * DEPRECATED: use pose::Position instead. Will be removed in GAMS v2
    **/
    class GAMS_EXPORT Position
    {
    public:
      /**
       * Constructor
       * @param  init_x    the x axis coordinate(e.g. latitude)
       * @param  init_y    the y axis coordinate(e.g. longitude)
       * @param  init_z    the z axis coordinate(e.g. altitude)
       **/
      Position(
        double init_x = DBL_MAX, double init_y = 0.0, double init_z = 0.0);

      /**
       * Copy constructor
       * @param source  the source to copy
       **/
      Position(const Position & source);

      Position(const pose::Position & pos)
        : x(pos.x()), y(pos.y()), z(pos.z()) {}

      pose::Position to_pos(const pose::ReferenceFrame &frame) const {
        return pose::Position(frame, x, y, z);
      }

      /**
       * Destructor
       **/
      virtual ~Position();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator=(const Position & rhs);

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator==(const Position & rhs) const;
      
      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator==(const
        madara::knowledge::containers::DoubleArray & rhs) const;

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator==(const
        madara::knowledge::containers::NativeDoubleArray & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!=(const Position & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!=(const
        madara::knowledge::containers::DoubleArray & rhs) const;

      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!=(const
        madara::knowledge::containers::NativeDoubleArray & rhs) const;

      /**
       * Less than used for ordering in stl containers
       * @param rhs   comparing position
       * @return true if *this is less than rhs
       **/
      bool operator<(const Position& rhs) const;

      /**
       * Dot product of two positions treated as vectors
       * @param p_2   other position
       * @return dot product of the two positions
       **/
      virtual double dot(const Position& p_2) const;

      /**
       * Approximate equality
       * @param  rhs      value to compare
       * @param  epsilon  approximation value
       * @return true if position is within epsilon in each direction of this
       **/
      bool approximately_equal(
        const Position & rhs, const double & epsilon) const;

      /**
       * Approximate equality
       * @param  rhs      value to compare
       * @param  epsilon  approximation value
       * @return true if position is within epsilon in each direction of this
       **/
      bool approximately_equal_2d(
        const Position & rhs, const double & epsilon) const;

      /**
       * Get spherical direction to position
       * @param rhs     other position
       * @param phi     direction in x/y plane
       * @param theta   direction in z plane
       **/
      void direction_to(
        const Position& rhs, double& phi, double& theta) const;

      /**
       * Get distance between two positions
       * @param  rhs      second position
       * @return euclidean distance between the two points
       **/
      double distance_to(const Position & rhs) const;

      /**
       * @param  rhs      second position
       * @return euclidean distance between the two points with just x and y
       **/
      virtual double distance_to_2d(const Position & rhs) const;

      /**
       * Get distance between a point and a line segment
       * @param end   end of line segment including *this
       * @param check point to find distance for
       * @return distance from check to line including *this and end
       **/
      virtual double distance_to_2d(
        const Position& end, const Position& check) const;

      /**
       * Get slope between two points
       * @param p     other point
       * @param slope location to store slope between two points
       * @return      true if slope exists
       **/
      virtual bool slope_2d(
        const Position & p, double & slope) const;

      /**
       * Deterime if a third point is inline and between another two points
       * @param end     second endpoint
       * @param check   point to check
       * @return true if check is inline and between *this and end
       **/
      virtual bool is_between_2d(const Position & end, const Position & check) const;
      
      /**
       * Helper function for converting the position to a string
       * @param delimiter characters to insert between position components
       **/
      std::string to_string(const std::string & delimiter = ",") const;

      /**
       * Helper function for creating a Position from a string
       * @param delimiter characters to insert between position components
       **/
      static Position from_string(const std::string & delimiter = ",");

      /**
      * Helper function for creating a Position from a KnowledgeRecord
      * @param record  a record containing a double array
      * @return  the Position equivalent of the record
      **/
      static Position from_record(const madara::knowledge::KnowledgeRecord & record);

      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container(
        madara::knowledge::containers::DoubleArray & target) const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container(
        madara::knowledge::containers::DoubleArray & source);
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container(
        madara::knowledge::containers::NativeDoubleArray & target)
        const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container(
        madara::knowledge::containers::NativeDoubleArray & source);

      /// the x coordinate(e.g. latitude)
      double x;

      /// the y coordinate(e.g. longitude)
      double y;

      /// the z coordinate(e.g. altitude)
      double z;

    protected:
      /**
       * Subtraction operator performs element-wise subtraction
       * @param rhs   value to subtract from *this
       * @return element-wise subtraction of rhs from *this
       **/
      Position operator-(const Position & rhs) const;

      /**
       * Addition operator performs element-wise addition
       * @param rhs   value to add to *this
       * @return element-wise addition of *this and rhs
       **/
      Position operator+(const Position & rhs) const;

      /**
       * Scale the position
       * @param scale   factor to scale by
       * @return scaled version of *this
       **/
      Position operator*(const double& scale) const;
    };

    // helpful typedef for vector of positions
    typedef std::vector <Position>    Positions;
  }
}

#endif // _GAMS_UTILITY_POSITION_H_
