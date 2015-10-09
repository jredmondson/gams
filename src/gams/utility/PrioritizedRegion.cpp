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
 * @file PrioritizedRegion.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized region associates a priority with a region
 **/

#include "gams/utility/PrioritizedRegion.h"

#include <vector>
#include <string>
#include <sstream>

#include "madara/knowledge/containers/Integer.h"

#include "gams/loggers/GlobalLogger.h"

using std::string;
using std::vector;

gams::utility::PrioritizedRegion::PrioritizedRegion (
  const vector <GPSPosition> & init_points, const unsigned int p, 
  const std::string& name) :
  Region (init_points, 0, name), priority (p)
{
}

gams::utility::PrioritizedRegion::PrioritizedRegion (const Region & region,
  const unsigned int p, const std::string& name) :
  Region (region), priority (p)
{
  set_name (name);
}

gams::utility::PrioritizedRegion::~PrioritizedRegion ()
{
}

void
gams::utility::PrioritizedRegion::operator= (const PrioritizedRegion & rhs)
{
  if (this != &rhs)
  {
    this->Region::operator= (rhs);
    this->priority = rhs.priority;
  }
}

bool
gams::utility::PrioritizedRegion::operator== (const PrioritizedRegion& rhs) const
{
  return (this == &rhs) || 
    (((Region*)this)->operator==(rhs) && (priority == rhs.priority));
}

bool
gams::utility::PrioritizedRegion::operator!= (const PrioritizedRegion& rhs) const
{
  return !(*this == rhs);
}

std::string
gams::utility::PrioritizedRegion::to_string (const std::string & delimiter)
  const
{
  std::stringstream ret_val;
  ret_val << this->Region::to_string (delimiter) << std::endl;
  ret_val << "\tpriority: " << priority << std::endl;
  return ret_val.str ();
}

bool
gams::utility::PrioritizedRegion::check_valid_type (
  madara::knowledge::KnowledgeBase& kb, const std::string& name) const
{
  const static Class_ID valid = 
    (Class_ID) (REGION_TYPE_ID | PRIORITIZED_REGION_TYPE_ID);
  return Containerize::is_valid_type (kb, name, valid);
}

void
gams::utility::PrioritizedRegion::to_container_impl (
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  Region temp (*this);
  temp.to_container (kb, name);

  madara::knowledge::containers::Integer object_type;
  object_type.set_name (name + object_type_suffix_, kb);
  object_type = PRIORITIZED_REGION_TYPE_ID;

  madara::knowledge::containers::Integer priority_container;
  priority_container.set_name (name + ".priority", kb);
  priority_container = priority;
}

bool
gams::utility::PrioritizedRegion::from_container_impl (
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  bool ret_val (false);
  if (!check_valid_type (kb, name))
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::PrioritizedRegion::from_container:" \
      " \"%s\" is not a valid Region\n", name.c_str ());
    ret_val = false;
  }
  else
  {
    Region temp_reg;
    ret_val = temp_reg.from_container (kb, name);
    if (ret_val)
    {
      madara::knowledge::containers::Integer priority_container;
      priority_container.set_name (name + ".priority", kb);
      if (!priority_container.exists () && 
        get_type (kb, name) != REGION_TYPE_ID)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::utility::PrioritizedRegion::from_container:" \
          " \"%s\" is missing priority value\n", name.c_str ());
        ret_val = false;
      }
      else if (get_type (kb, name) == REGION_TYPE_ID)
      {
        priority_container = 1; // default to 1
      }

      if (ret_val)
      {
        operator= (
          PrioritizedRegion (temp_reg, priority_container.to_integer ()));
      }
    }
  }
  return ret_val;
}
