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
 * @file test_control_loop.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a test driver for the GAMS controllers loop.
 **/

#include "madara/knowledge/KnowledgeBase.h"
#include "gams/controllers/MapeLoop.h"

// create shortcuts to MADARA classes and namespaces
namespace engine = madara::knowledge;
namespace controllers = gams::controllers;
typedef madara::KnowledgeRecord   Record;
typedef Record::Integer Integer;

/**
 * Monitor function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record monitor (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".monitor");

  return Integer (0);
}

/**
 * Analyze function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record analyze (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".analyze");

  return Integer (0);
}

/**
 * Plan function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record plan (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  Record value = vars.inc (".plan");

  return Integer (value == Integer (20));
}

/**
 * Execute function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record execute (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".execute");

  return Integer (0);
}

// perform main logic of program
int main (int /*argc*/, char ** /*argv*/)
{
  // create knowledge base and a control loop
  engine::KnowledgeBase knowledge;
  controllers::MapeLoop loop (knowledge);

  // initialize variables and function stubs
  loop.init_vars (knowledge, 0, 4);
  loop.define_monitor (monitor);
  loop.define_analyze (analyze);
  loop.define_plan (plan);
  loop.define_execute (execute);

  // run a mape loop every 1s for 50s
  loop.run (1.0, 50.0);

  // print all knowledge values
  knowledge.print ();

  return 0;
}
