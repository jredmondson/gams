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
 * @file Cartesian_Frame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_CARTESIAN_FRAME_H_
#define _GAMS_UTILITY_CARTESIAN_FRAME_H_

#include "Reference_Frame.h"

namespace gams
{
  namespace utility
  {
    /**
     * Locations represented as meters distance, in x, y, and z, from an origin point
     * Rotations represented as Euler angles, in the origin frame, applied in this order:
     *    alpha degrees around X-axis
     *    beta  degrees around Y-axis
     *    gamma degrees around Z-axis
     **/
    class GAMS_Export Cartesian_Frame : public Axis_Angle_Frame
    {
    public:
      Cartesian_Frame() : Axis_Angle_Frame() {}
      explicit Cartesian_Frame(const Pose &origin) : Axis_Angle_Frame(origin) {}
      explicit Cartesian_Frame(Pose *origin) : Axis_Angle_Frame(origin) {}

    private:
      virtual std::string get_name() const { return "Cartesian"; }

      virtual void transform_to_origin(Location_Base &in) const;

      virtual void transform_from_origin(Location_Base &in) const;

      virtual double calc_distance(const Location_Base &loc1, const Location_Base &loc2) const
      {
        double x_dist = loc2.x() - loc1.x();
        double y_dist = loc2.y() - loc1.y();
        double z_dist = loc2.z() - loc1.z();

        return sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);
      }
    };
  }
}

#endif
