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
 * @file Containerize.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Defines is_correct_type function for Containerize
 **/

#include "gams/utility/Containerize.h"

const std::string gams::utility::Containerize::object_type_suffix_ (
  ".object_type");

gams::utility::Containerize::Class_ID
gams::utility::Containerize::get_type (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name)
{
  switch (kb.get (name + object_type_suffix_).to_integer ())
  {
    case REGION_TYPE_ID:
      return REGION_TYPE_ID;
    case PRIORITIZED_REGION_TYPE_ID:
      return PRIORITIZED_REGION_TYPE_ID;
    case SEARCH_AREA_TYPE_ID:
      return SEARCH_AREA_TYPE_ID;
  }
  return INVALID;
}

bool
gams::utility::Containerize::is_valid_type (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name, 
  const Class_ID& valid) const
{
  Madara::Knowledge_Record record = kb.get (name + object_type_suffix_);
  if (record.exists ())
    return (record.to_integer () & valid);
  return false;
}
