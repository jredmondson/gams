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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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
 * @file Prioritized_Region.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized region associates a priority with a region
 **/

#include "gams/utility/Prioritized_Region.h"

#include <vector>
#include <string>

using std::string;
using std::vector;

gams::utility::Prioritized_Region::Prioritized_Region (
  const vector <GPS_Position> & init_points, const unsigned int p) :
  Region (init_points), priority (p)
{
}

gams::utility::Prioritized_Region::Prioritized_Region (const Region & region,
  const unsigned int p) :
  Region (region), priority (p)
{
}

void
gams::utility::Prioritized_Region::operator= (const Prioritized_Region & rhs)
{
  if (this != &rhs)
  {
    this->Region::operator= (rhs);
    this->priority = rhs.priority;
  }
}

void
gams::utility::Prioritized_Region::init (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const string & prefix)
{
  // initialize super class variables
  ((Region *)(this))->init (knowledge, prefix);

  // get priority
  priority = knowledge.get (prefix + ".priority").to_integer ();
}

gams::utility::Prioritized_Region
gams::utility::parse_prioritized_region (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const string & prefix)
{
  Prioritized_Region result;
  result.init (knowledge, prefix);
  return result;
}
