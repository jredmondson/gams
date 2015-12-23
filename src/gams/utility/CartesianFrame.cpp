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

#include "CartesianFrame.h"
#include "GPSFrame.h"
#include "UTMFrame.h"

namespace gams
{
  namespace utility
  {
    void CartesianFrame::transform_location_to_origin(
                      double &x, double &y, double &z) const
    {
      GAMS_WITH_FRAME_TYPE(origin(), CartesianFrame, frame)
      {
        rotate_location_vec(x, y, z, origin());

        x += origin().x();
        y += origin().y();
        z += origin().z();
        return;
      }
      GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
      {
        rotate_location_vec(x, y, z, origin());

        z += origin().z();

        // convert the latitude/y coordinates
        y = (y * 360.0) / frame->circ() + origin().y();

        // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        double r_prime = frame->radius() * cos (DEG_TO_RAD (y));
        double circumference = 2 * r_prime * M_PI;
        x = (x * 360.0) / circumference + origin().x();

        frame->normalize_location(x, y, z);
        return;
      }
      throw undefined_transform(*this, origin().frame(), true);
    }

    void CartesianFrame::transform_location_from_origin(
                      double &x, double &y, double &z) const
    {
      GAMS_WITH_FRAME_TYPE(origin(), CartesianFrame, frame)
      {
        rotate_location_vec(x, y, z, origin(), true);

        x -= origin().x();
        y -= origin().y();
        z -= origin().z();
        return;
      }
      GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
      {
        if(!origin().is_rotation_zero())
          throw undefined_transform(*this, origin().frame(), false, true);
        frame->normalize_location(x, y, z);

        z -= origin().z();

        // convert the latitude/y coordinates
        double new_y = ((y - origin().y()) * frame->circ()) / 360.0;

        // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        double r_prime = frame->radius() * cos (DEG_TO_RAD (y));
        double circumference = 2 * r_prime * M_PI;
        y = new_y;
        x = ((x - origin().x()) * circumference) / 360.0;
        return;
      }
      throw undefined_transform(*this, origin().frame(), false);
    }

    namespace
    {
      CartesianFrame cartesian_default_frame;
    }

    const ReferenceFrame *CoordinateBase::default_frame_ =
           &cartesian_default_frame;
  }
}
