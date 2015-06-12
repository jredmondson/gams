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

#include "Cartesian_Frame.h"
#include "GPS_Frame.h"

namespace gams
{
  namespace utility
  {
    void Cartesian_Frame::transform_to_origin(Location_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin, Cartesian_Frame, frame)
      {
        in.x += origin.x;
        in.y += origin.y;
        in.z += origin.z;
        return;
      }
      GAMS_WITH_FRAME_TYPE(origin, GPS_Frame, frame)
      {
        in.z += origin.z;

        // convert the latitude/y coordinates
        in.y = (in.y * 360.0) / frame.get_circ() + origin.y;

        // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        double r_prime = frame.get_radius() * cos (DEG_TO_RAD (in.y));
        double circumference = 2 * r_prime * M_PI;
        in.x = (in.x * 360.0) / circumference + origin.x;

        frame.normalize(in);
        return;
      }
      throw undefined_transform(*this, origin.get_frame(), true);
    }

    void Cartesian_Frame::transform_from_origin(Location_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin, Cartesian_Frame, frame)
      {
        in.x -= origin.x;
        in.y -= origin.y;
        in.z -= origin.z;
        return;
      }
      GAMS_WITH_FRAME_TYPE(origin, GPS_Frame, frame)
      {
        frame.normalize(in);

        // convert the latitude/y coordinates
        in.y = ((in.y - origin.y) * frame.get_circ()) / 360.0;

        // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        double r_prime = frame.get_radius() * cos (DEG_TO_RAD (in.y));
        double circumference = 2 * r_prime * M_PI;
        in.x = ((in.x - origin.x) * circumference) / 360.0;
        return;
      }
      throw undefined_transform(*this, origin.get_frame(), false);
    }

    void Cartesian_Frame::transform_to_origin(Rotation_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin, Cartesian_Frame, frame)
      {
        Quaternion in_quat(in);
        Quaternion origin_quat(static_cast<const Rotation_Base &>(origin));
        in_quat *= origin_quat;
        in_quat.to_rotation_vector(in);
        return;
      }
      throw undefined_transform(*this, origin.get_frame(), true);
    }

    void Cartesian_Frame::transform_from_origin(Rotation_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin, Cartesian_Frame, frame)
      {
        Quaternion in_quat(in);
        Quaternion origin_quat(static_cast<const Rotation_Base &>(origin));
        origin_quat.conjugate();
        in_quat *= origin_quat;
        in_quat.to_rotation_vector(in);
        return;
      }
      throw undefined_transform(*this, origin.get_frame(), false);
    }

    Cartesian_Frame cartesian_default_frame;

    namespace __INTERNAL__
    {
      Base_Frame *default_frame = &cartesian_default_frame;
    }
  }
}
