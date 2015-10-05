/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Axes.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a utility class for working with position
 **/

#ifndef _GAMS_UTILITY_AXES_H_
#define _GAMS_UTILITY_AXES_H_

#include <vector>

#include "gams/GAMS_Export.h"
#include "madara/knowledge/containers/Double_Vector.h"
#include "madara/knowledge/containers/Native_Double_Vector.h"

namespace gams
{
  namespace utility
  {
    /**
    * A deprecated class for rotation information. @see Reference_Frame
    **/
    class GAMS_Export Axes
    {
    public:
      /**
       * Constructor
       * @param  init_x    the x axis rotation
       * @param  init_y    the y axis rotation
       * @param  init_z    the z axis rotation
       **/
      Axes (
        double init_x = 0.0, double init_y = 0.0, double init_z = 0.0);

      /**
       * Copy constructor
       * @param source  the source to copy
       **/
      Axes(const Axes & source);

      /**
       * Destructor
       **/
      virtual ~Axes ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Axes & rhs);

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator== (const Axes & rhs) const;
      
      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator== (const
        madara::knowledge::containers::Double_Array & rhs) const;

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator== (const
        madara::knowledge::containers::Native_Double_Array & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!= (const Axes & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!= (const
        madara::knowledge::containers::Double_Array & rhs) const;

      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if x, y, z are equal in both objects, false otherwise
       **/
      bool operator!= (const
        madara::knowledge::containers::Native_Double_Array & rhs) const;

      /**
       * Helper function for converting the position to a string
       * @param delimiter characters to insert between position components
       **/
      virtual std::string to_string (const std::string & delimiter = ",") const;

      /**
       * Helper function for creating a Axes from a string
       * @param delimiter characters to insert between position components
       **/
      static Axes from_string (const std::string & delimiter = ",");

      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container (
        madara::knowledge::containers::Double_Array & target) const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container (
        madara::knowledge::containers::Double_Array & source);
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container (
        madara::knowledge::containers::Native_Double_Array & target)
        const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container (
        madara::knowledge::containers::Native_Double_Array & source);

      /// the x axis angle
      double x;

      /// the y axis angle
      double y;

      /// the z axis angle
      double z;

    protected:
      /**
       * Subtraction operator performs element-wise subtraction
       * @param rhs   value to subtract from *this
       * @return element-wise subtraction of rhs from *this
       **/
      Axes operator- (const Axes & rhs) const;

      /**
       * Addition operator performs element-wise addition
       * @param rhs   value to add to *this
       * @return element-wise addition of *this and rhs
       **/
      Axes operator+ (const Axes & rhs) const;

      /**
       * Scale the position
       * @param scale   factor to scale by
       * @return scaled version of *this
       **/
      Axes operator* (const double& scale) const;
    };
  }
}

#endif // _GAMS_UTILITY_AXES_H_
