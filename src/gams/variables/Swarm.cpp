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
#include "Swarm.h"

#include <string>

using std::string;

typedef  madara::knowledge::KnowledgeRecord::Integer  Integer;

gams::variables::Swarm::Swarm ()
{
}

gams::variables::Swarm::~Swarm ()
{
}

void
gams::variables::Swarm::operator= (const Swarm & rhs)
{
  if (this != &rhs)
  {
    this->accents = rhs.accents;
    this->algorithm = rhs.algorithm;
    this->algorithm_id = rhs.algorithm_id;
    this->algorithm_args = rhs.algorithm_args;
    this->min_alt = rhs.min_alt;
    this->size = rhs.size;
  }
}


void
gams::variables::Swarm::init_vars (
  madara::knowledge::KnowledgeBase & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer & swarm_size)
{
  // initialize the variable containers
  variables::init_vars (accents, knowledge, "swarm");
  min_alt.set_name ("swarm.min_alt", knowledge);
  algorithm.set_name ("swarm.algorithm", knowledge);
  algorithm_id.set_name ("swarm.algorithm.id", knowledge);
  algorithm_args.set_name ("swarm.algorithm.args", knowledge);
  size.set_name ("swarm.size", knowledge);

  init_vars (swarm_size);
}

void
gams::variables::Swarm::init_vars (
  madara::knowledge::Variables & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& swarm_size)
{
  // initialize the variable containers
  variables::init_vars (accents, knowledge, "swarm");
  min_alt.set_name ("swarm.min_alt", knowledge);
  algorithm.set_name ("swarm.algorithm", knowledge);
  algorithm_id.set_name ("swarm.algorithm.id", knowledge);
  algorithm_args.set_name ("swarm.algorithm.args", knowledge);
  size.set_name ("swarm.size", knowledge);

  init_vars (swarm_size);
}

void gams::variables::Swarm::init_vars (
  const madara::knowledge::KnowledgeRecord::Integer& swarm_size)
{
  madara::knowledge::KnowledgeUpdateSettings defaults;
  madara::knowledge::KnowledgeUpdateSettings keep_local (true);

  // keep certain varaible changes as local only
  min_alt.set_settings (keep_local);
  algorithm.set_settings (keep_local);
  algorithm_id.set_settings (keep_local);
  algorithm_args.set_settings (keep_local);
  size.set_settings (keep_local);

  if (swarm_size >= 0)
  {
    // update swarm size
    size = swarm_size;
  }

  // use default settings
  size.set_settings (defaults);
}

void gams::variables::init_vars (Swarm & variables,
  madara::knowledge::KnowledgeBase & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& swarm_size)
{
  variables.init_vars (knowledge, swarm_size);
}

void gams::variables::init_vars (Swarm & variables,
  madara::knowledge::Variables & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& swarm_size)
{
  variables.init_vars (knowledge, swarm_size);
}
