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
 *      are those of the author (s) and do not necessarily reflect the views of
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
 * @file Orientation.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Orientation and OrientationVector classes
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_ORIENTATION_H
#define _GAMS_POSE_ORIENTATION_H

#include <iostream>
#include <string>
#include <gams/CPP11_compat.h>
#include <gams/pose/Coordinate.h>
#include <gams/pose/Angular.h>
#include "gams/GAMSExport.h"
#include <cmath>
#include <madara/knowledge/containers/DoubleVector.h>
#include <madara/knowledge/containers/NativeDoubleVector.h>

#define DEG_TO_RAD(x) ( ( (x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) ( ( (x) * 180) / M_PI)

#include "AngleUnits.h"

namespace gams
{
  namespace pose
  {
    class ReferenceFrame;

    class Quaternion;
    /**
     * Represents a orientation or orientation within a reference frame.
     * Uses axis-angle notation: a orientation is represented by a vector whose
     * direction forms the axis of orientation, with angle of orientation equal to
     * length of the vector.
     *
     * All orientations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the orientation axis, orientations curve in the direction your
     * fingers are pointing.
     **/
    class GAMSExport Orientation : public Angular<Orientation>
    {
    public:
      using Angular::Angular;
    };
  }
}

#include "Orientation.inl"

#endif // _GAMS_UTILITY_ORIENTATION_H_
