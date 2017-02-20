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
 * @file Pose.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains functions for the Pose class
 **/

#include "Pose.h"
#include "ReferenceFrame.h"

void gams::utility::Pose::to_container (
  madara::knowledge::containers::NativeDoubleVector &container) const
{
  if (frame ().name () == "GPS")
  {
    container.set (0, get (order::GPS::find (0)));
    container.set (1, get (order::GPS::find (1)));
    container.set (2, get (order::GPS::find (2)));
  }
  else
  {
    container.set (0, get (order::XYZ::find (0)));
    container.set (1, get (order::XYZ::find (1)));
    container.set (2, get (order::XYZ::find (2)));
  }
}


void gams::utility::Pose::from_container (
  const madara::knowledge::containers::NativeDoubleVector &container)
{
  if (frame ().name () == "GPS")
  {
    set (0, container[order::GPS::get (0)]);
    set (1, container[order::GPS::get (1)]);
  }
  else
  {
    set (0, container[order::XYZ::get (0)]);
    set (1, container[order::XYZ::get (1)]);
  }

  for (int i = 2; i < container.size (); ++i)
  {
    set (i, container[i]);
  }

  if (container.size () <= 3)
  {
    if (container.size () == 2)
    {
      set (2, 0);
    }

    set (3, 0);
    set (4, 0);
    set (5, 0);
  }
}

void gams::utility::Pose::from_container (
  const std::vector <double> &container)
{
  if (frame ().name () == "GPS")
  {
    set (0, container[order::GPS::get (0)]);
    set (1, container[order::GPS::get (1)]);
  }
  else
  {
    set (0, container[order::XYZ::get (0)]);
    set (1, container[order::XYZ::get (1)]);
  }

  for (int i = 2; i < container.size (); ++i)
  {
    set (i, container[i]);
  }

  if (container.size () <= 3)
  {
    if (container.size () == 2)
    {
      set (2, 0);
    }

    set (3, 0);
    set (4, 0);
    set (5, 0);
  }
}
