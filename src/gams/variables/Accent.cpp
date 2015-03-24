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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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
#include "Accent.h"

#include <string>

gams::variables::Accent::Accent ()
{
}

gams::variables::Accent::~Accent ()
{
}

void
gams::variables::Accent::operator= (const Accent & accent)
{
  if (this != &accent)
  {
    this->command = accent.command;
    this->command_args = accent.command_args;
  }
}

void
gams::variables::Accent::init_vars (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const std::string & prefix)
{
  std::string accent_name (prefix);
  accent_name += ".accent";

  // initialize the variable containers
  command.set_name (accent_name, knowledge);
  command_args.set_name (accent_name, knowledge);

  // init settings
  init_variable_settings ();
}

void
gams::variables::Accent::init_vars (
  Madara::Knowledge_Engine::Variables & knowledge,
  const std::string & prefix)
{
  std::string accent_name (prefix);
  accent_name += ".accent";

  // initialize the variable containers
  command.set_name (accent_name, knowledge);
  command_args.set_name (accent_name, knowledge);

  // init settings
  init_variable_settings ();
}

void gams::variables::init_vars (Accents & variables,
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const std::string & prefix)
{
  std::string accent_name (prefix);
  accent_name += ".accent";

  Madara::Knowledge_Record::Integer size =
    knowledge.get (accent_name + ".size").to_integer ();
  
  // iterate through all accents
  for (unsigned int i = 0; i < size; ++i)
  {
    // each accent is at prefix.{i}
    std::stringstream buffer (accent_name);
    buffer << ".";
    buffer << i;

    variables[i].init_vars (knowledge, buffer.str ());
  }
}

void gams::variables::init_vars (Accents & variables,
  Madara::Knowledge_Engine::Variables & knowledge,
  const std::string & prefix)
{
  std::string accent_name (prefix);
  accent_name += ".accent";

  Madara::Knowledge_Record::Integer size =
    knowledge.get (accent_name + ".size").to_integer ();
  
  // iterate through all accents
  for (unsigned int i = 0; i < size; ++i)
  {
    // each accent is at prefix.{i}
    std::stringstream buffer (accent_name);
    buffer << ".";
    buffer << i;

    variables[i].init_vars (knowledge, buffer.str ());
  }
}


void
gams::variables::Accent::init_variable_settings ()
{
  // keep certain varaible changes as local only
  Madara::Knowledge_Engine::Knowledge_Update_Settings keep_local (true);
  command.set_settings (keep_local);
  command_args.set_settings (keep_local);
}
