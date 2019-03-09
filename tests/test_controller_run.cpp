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

#include <iostream>
#include <iomanip>

#include "madara/knowledge/KnowledgeBase.h"
#include "gams/controllers/BaseController.h"

#include "helper/CounterAlgorithm.h"
#include "helper/CounterPlatform.h"

#include "madara/utility/Utility.h"

// default transport settings
std::string host ("");
const std::string default_multicast ("239.255.0.1:4150");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace engine = madara::knowledge;
namespace controllers = gams::controllers;
namespace platforms = gams::platforms;
namespace algorithms = gams::algorithms;

typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

// global variables for platform and algorithm
platforms::CounterPlatform * platform (0);
algorithms::CounterAlgorithm * algorithm (0);
  
std::vector <double> normal_blasting;
std::map <double, Integer> normal_loops;
std::map <double, std::string> normal_status;

int gams_fails = 0;

Record
to_legible_hertz (engine::FunctionArguments & args, engine::Variables & /*vars*/)
{
  Record result;

  if (args.size () == 1)
  {
    std::stringstream buffer;

    std::locale loc ("C"); 
    buffer.imbue (loc); 

    const int ghz_mark = 1000000000;
    const int mhz_mark = 1000000;
    const int khz_mark = 1000;

    double hertz = args[0].to_double ();

    double freq = hertz / ghz_mark;

    if (freq >= 1)
    {
      buffer << std::setprecision (2) << std::fixed;
      buffer << freq;
      buffer << " ghz";
      result = Record (buffer.str ().c_str ());
    }
    else
    {
      freq = hertz / mhz_mark;

      if (freq >= 1)
      {
        buffer << std::setprecision (2) << std::fixed;
        buffer << freq;
        buffer << " mhz";
        result = Record (buffer.str ().c_str ());
      }
      else
      {
        freq = hertz / khz_mark;

        if (freq >= 1)
        {
          buffer << std::setprecision (2) << std::fixed;
          buffer << freq;
          buffer << " khz";
          result = Record (buffer.str ().c_str ());
        }
        else
        {
          freq = hertz;

          buffer << std::setprecision (2) << std::fixed;
          buffer << freq;
          buffer << " hz";
          result = Record (buffer.str ().c_str ());
        }
      }
    }
  }
  
  return result;
}

void
test_period (engine::KnowledgeBase & knowledge,
  controllers::BaseController & loop, double period = 0.0, double duration = 10.0)
{
  algorithm->reset_counters ();
  std::cerr << "Testing " << duration << "s experiment with "
    << period << "s period.\n";
  loop.run (period, duration);
  knowledge.set (".loops", algorithm->loops);


#ifndef _MADARA_NO_KARL_
  knowledge.evaluate (".loop_speed = .loops / 10");
  knowledge.evaluate (".legible_speed = to_legible_hertz (.loop_speed)");
#else
  knowledge.set (".loop_speed", algorithm->loops / 10);
#endif
  knowledge.print (
    "  Results: {.loops} loop executions @ {.legible_speed}\n");

  if (period == 0.0 || ((double)algorithm->loops > (1/period) * .1 * duration)
      || algorithm->loops > 100)
  {
    knowledge.print (
      "  SUCCESS: {.loops} > hz * .1 * duration\n");
  }
  else
  {
    knowledge.print (
      "  FAIL: {.loops} < hz * .1 * duration\n");
    ++gams_fails;
  }
}

void
test_hz (engine::KnowledgeBase & knowledge,
  controllers::BaseController & loop, double hz = 0.0, double duration = 10.0)
{
  algorithm->reset_counters ();
  std::cerr << "Testing " << duration << "s experiment with "
    << hz << "hz.\n";
  loop.run_hz (hz, duration);
  knowledge.set (".loops", algorithm->loops);

#ifndef _MADARA_NO_KARL_
  knowledge.evaluate (".loop_speed = .loops / 10");
  knowledge.evaluate (".legible_speed = to_legible_hertz (.loop_speed)");
#else
  knowledge.set (".loop_speed", algorithm->loops / 10);
#endif
  knowledge.print (
    "  Results: {.loops} loop executions @ {.legible_speed}\n");

  if (hz == 0.0 || ((double)algorithm->loops > hz * .1 * duration) || 
      algorithm->loops > 100)
  {
    knowledge.print (
      "  SUCCESS: {.loops} > hz * .1 * duration\n");
  }
  else
  {
    knowledge.print (
      "  FAIL: {.loops} < hz * .1 * duration\n");
    ++gams_fails;
  }
}

// perform main logic of program
int main (int /*argc*/, char ** /*argv*/)
{
  // create knowledge base and a control loop
  engine::KnowledgeBase knowledge;
  controllers::BaseController loop (knowledge);

  platform =
    new platforms::CounterPlatform (knowledge);
  algorithm =
    new algorithms::CounterAlgorithm (knowledge);
  
  // set our priority to the default high priority
  madara::utility::set_thread_priority ();

#ifndef _MADARA_NO_KARL_
  knowledge.define_function ("to_legible_hertz", to_legible_hertz);
#endif

  // initialize variables and function stubs
  loop.init_vars (0, 4);
  
  algorithm->disable_counters ();
  
  // initialize the platform and algorithm
  loop.init_platform (platform);
  loop.init_algorithm (algorithm);

  double period = 0.0;
  double hertz = 0.0;
  double duration = 10.0;

  std::cerr << "*****************************************************\n";
  std::cerr <<
    "* Running single non-MADARA counter in algorithm->analyze ()\n";
  std::cerr << "*****************************************************\n";

  // run blasting experiments
  test_hz (knowledge, loop);
  test_period (knowledge, loop);
  
  // run specific periods and hertz
  for (period = 0.1, hertz = 10.0; hertz <= 1000000.0; period /= 10, hertz *= 10)
  {
    test_hz (knowledge, loop, hertz, duration);
    test_period (knowledge, loop, period, duration);
  }
  
  std::cerr << "*****************************************************\n";
  std::cerr <<
    "* Running MADARA counters in algorithm: analyze, plan, execute\n";
  std::cerr << "*****************************************************\n";

  algorithm->enable_counters ();

  // run blasting experiments
  test_hz (knowledge, loop);
  test_period (knowledge, loop);

  // run specific periods and hertz
  for (period = 0.1, hertz = 10.0; hertz <= 1000000.0; period /= 10, hertz *= 10)
  {
    test_hz (knowledge, loop, hertz, duration);
    test_period (knowledge, loop, period, duration);
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
