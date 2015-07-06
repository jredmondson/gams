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

#ifndef _GAMS_UTILITY_GPS_FRAME_INL_
#define _GAMS_UTILITY_GPS_FRAME_INL_

#include "Reference_Frame.h"

namespace gams
{
  namespace utility
  {
    inline GPS_Frame::GPS_Frame(double planet_radius)
      : Axis_Angle_Frame(), _planet_radius(planet_radius) {}

    inline GPS_Frame::GPS_Frame(
          const Pose &origin,
          double planet_radius)
      : Axis_Angle_Frame(origin), _planet_radius(planet_radius) {}

    inline GPS_Frame::GPS_Frame(
          Pose *origin,
          double planet_radius)
      : Axis_Angle_Frame(origin), _planet_radius(planet_radius) {}

    inline double GPS_Frame::radius() const
    {
      return _planet_radius;
    }

    inline double GPS_Frame::circ() const
    {
      return 2 * _planet_radius * M_PI;
    }

    inline double GPS_Frame::radius(double new_radius)
    {
      return _planet_radius = new_radius;
    }

    inline double GPS_Frame::circ(double new_circ)
    {
      return _planet_radius = new_circ / (2 * M_PI);
    }

    inline std::string GPS_Frame::get_name() const
    {
      return "GPS";
    }
  }
}

#endif
