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
 * @file ReferenceFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_REFERENCE_FRAME_H_
#define _GAMS_UTILITY_REFERENCE_FRAME_H_

#include "gams/pose/ReferenceFrame.h"
#include "gams/utility/Pose.h"

namespace gams
{
  namespace utility
  {
    /**
     * Thrown when a reference frame function is called with a Coordinate
     * type (e.g., Pose, Location, Orientation) that frame does not support.
     *
     * @tparam CoordType The kind of Coordinate the error was raised for.
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    using bad_coord_type = gams::pose::bad_coord_type;

    /**
     * Thrown when an an attempt is made to transform between frames
     * that do not belong to the same frame tree.
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    typedef gams::pose::unrelated_frames unrelated_frames;

    /**
     * Thrown when an attempt is made to transform between two frame types,
     * and there's not transform defined.
     *
     * Note that between two frame types A and B, there are four different
     * kinds of transforms, which must be defined individually:
     *    1) From A as parent to B as its child
     *    2) From A as child to B as its parent
     *    3) From B as parent to A as its child
     *    4) From B as child to A as its parent
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    typedef gams::pose::undefined_transform undefined_transform;

    /**
     * Base class for Reference Frames.
     * Inherit from this class if implementing a new reference frame.
     * Otherwise, do not use directly.
     *
     * If implementing a new reference frame, you will need to modify the
     * transform_to_origin and transform_from_origin methods of the reference
     * frames the new reference frame should be able to transform to and from.
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    typedef gams::pose::ReferenceFrame ReferenceFrame;
  }
}

#endif
