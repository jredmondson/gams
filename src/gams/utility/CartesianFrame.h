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
 * @file CartesianFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_CARTESIAN_FRAME_H_
#define _GAMS_UTILITY_CARTESIAN_FRAME_H_

#include "ReferenceFrame.h"

namespace gams
{
  namespace utility
  {
    /**
     * Locations represented as meters distance, in x, y, and z, from an origin
     * Orientations represented in Axis Angle notation
     *
     * All conversions to/from child and parent CartesianFrames are supported.
     * Conversions to/from a parent GPSFrame are supported, except converting
     * GPSFrame to a child CartesianFrame that is orientd w.r.t. the GPSFrame
     * Converting to GPSFrame from a orientd child Cartesian is supported.
     **/
    class GAMSExport CartesianFrame : public SimpleRotateFrame
    {
    public:
      /**
       * Default constructor. No parent frame.
       **/
      CartesianFrame();

      /**
       * Creates a copy of the origin Pose passed in.
       **/
      explicit CartesianFrame(const Pose &origin);

      /**
       * Uses an existing Pose as origin, and maintains
       * a pointer to it. Changes to it affect this frame
       **/
      explicit CartesianFrame(Pose *origin);

    protected:
      /**
       * Returns the name of this type of reference frame.
       *
       * @return the string "Cartesian"
       **/
      virtual std::string get_name() const;

      /**
       * Transforms a location to origin
       * @param x   the x coordinate
       * @param y   the y coordinate
       * @param z   the z coordinate
       **/
      virtual void transform_location_to_origin(
                      double &x, double &y, double &z) const;

      /**
      * Transforms a location from origin
      * @param x   the x coordinate
      * @param y   the y coordinate
      * @param z   the z coordinate
      **/
      virtual void transform_location_from_origin(
                      double &x, double &y, double &z) const;

      /**
      * Calculates distance from one point to another
      * @param x1   the x coordinate of the first point
      * @param y1   the y coordinate of the first point
      * @param z1   the z coordinate of the first point
      * @param x2   the x coordinate of the second point
      * @param y2   the y coordinate of the second point
      * @param z2   the z coordinate of the second point
      **/
      virtual double calc_distance(
                      double x1, double y1, double z1,
                      double x2, double y2, double z2) const;
    };
  }
}

#include "CartesianFrame.inl"

#endif
