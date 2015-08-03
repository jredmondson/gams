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

#include "gams/loggers/Global_Logger.h"

const std::string gams::utility::Containerize::object_type_suffix_ (
  ".object_type");

gams::utility::Containerize::Containerize (const std::string& n) :
  name_ (n), prev_kb_ (0)
{
}

gams::utility::Containerize::~Containerize ()
{
}

std::string
gams::utility::Containerize::get_name () const
{
  return name_;
}

void
gams::utility::Containerize::set_name (const std::string& n)
{
  name_ = n;
}

void
gams::utility::Containerize::set_knowledge_base (
  Madara::Knowledge_Engine::Knowledge_Base* kb)
{
  if (kb == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::set_knowledge_base:" \
      " attempted to set knowledge_base to 0\n");
  }
  else
  {
    prev_kb_ = kb;
  }
}

void
gams::utility::Containerize::modify ()
{
  bool error = false;
  if (prev_kb_ == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::modify:" \
      " invalid prev_kb_ to execute modify\n");
    error = true;
  }
  if (name_.empty ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::modify:" \
      " modify cannot be called until name_ is set\n");
    error = true;
  }
  
  if (!error)
  {
    to_container (*prev_kb_, name_);
  }
} 

void
gams::utility::Containerize::to_container (const std::string& name)
{
  if (prev_kb_ == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::to_container:" \
      " invalid prev_kb_ to execute to_container\n");
  }
  else
  {
    if (name.empty ())
    {
      if (name_.empty ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::utility::Containerize::to_container:" \
          " no valid name available\n");
      }
      else
      {
        to_container (*prev_kb_, name_);
      }
    }
    else
    {
      to_container (*prev_kb_, name);
    }
  }
}

void
gams::utility::Containerize::to_container (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name)
{
  if (name.empty () && name_.empty ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::to_container:" \
      " no valid name available\n");
  }
  else
  {
    prev_kb_ = &kb;
    to_container_impl (*prev_kb_, (name.empty ()) ? name_ : name);
  }
}

bool
gams::utility::Containerize::from_container (const std::string& name)
{
  if (prev_kb_ == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::from_container:" \
      " invalid prev_kb_\n");
  }
  else
  {
    if (!name.empty ())
      name_ = name;

    if (name_.empty ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::utility::Containerize::from_container:" \
        " no valid name available\n");
    }
    else
    {
      from_container (*prev_kb_, name_);
    }
  }
}

bool
gams::utility::Containerize::from_container (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name)
{
  bool ret_val (false);
  if (name.empty () && name_.empty ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Containerize::from_container:" \
      " no valid name available\n");
  }
  else
  {
    ret_val = from_container_impl (kb, (name.empty ()) ? name_ : name);
  }
  return ret_val;
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
