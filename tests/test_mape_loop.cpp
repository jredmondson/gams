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
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

int gams_fails = 0;

/**
 * Monitor function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record monitor (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".monitor");

  return Record (0);
}

/**
 * Analyze function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record analyze (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".analyze");

  return Record (0);
}

/**
 * Plan function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record plan (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  Record value = vars.inc (".plan");

  return Record (value == Integer (20));
}

/**
 * Execute function
 * @param  args   arguments to the function
 * @param  vars   interface to the knowledge base
 **/
Record execute (engine::FunctionArguments & /*args*/, engine::Variables & vars)
{
  vars.inc (".execute");

  return Record (0);
}

// perform main logic of program
int main (int /*argc*/, char ** /*argv*/)
{
  // create knowledge base and a control loop
  engine::KnowledgeBase knowledge;
  controllers::MapeLoop loop (knowledge);

  knowledge.print ("Defining MAPE functions\n");

  // initialize variables and function stubs
  loop.init_vars (knowledge, 0, 4);
  loop.define_monitor (monitor);
  loop.define_analyze (analyze);
  loop.define_plan (plan);
  loop.define_execute (execute);

  knowledge.print ("Looping at 1hz for 50s\n");

  // run a mape loop every 1s for 50s
  loop.run (1.0, 50.0);

  // print all knowledge values
  knowledge.print ();

  if (knowledge.get (".monitor").to_double () >= 10)
  {
    knowledge.print ("SUCCESS: {.monitor} monitor() is enough to pass\n");
  }
  else
  {
    knowledge.print ("FAIL: {.monitor} monitor() is not enough to pass\n");
    ++gams_fails;
  }
  
  if (knowledge.get (".analyze").to_double () >= 10)
  {
    knowledge.print ("SUCCESS: {.analyze} analyze() is enough to pass\n");
  }
  else
  {
    knowledge.print ("FAIL: {.analyze} analyze() is not enough to pass\n");
    ++gams_fails;
  }
  
  if (knowledge.get (".plan").to_double () >= 10)
  {
    knowledge.print ("SUCCESS: {.plan} plan() is enough to pass\n");
  }
  else
  {
    knowledge.print ("FAIL: {.plan} plan() is not enough to pass\n");
    ++gams_fails;
  }
  
  if (knowledge.get (".execute").to_double () >= 10)
  {
    knowledge.print ("SUCCESS: {.execute} execute() is enough to pass\n");
  }
  else
  {
    knowledge.print ("FAIL: {.execute} execute() is not enough to pass\n");
    ++gams_fails;
  }
  
  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}
