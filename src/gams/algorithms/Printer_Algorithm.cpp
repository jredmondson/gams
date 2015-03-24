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

#include "Printer_Algorithm.h"

#include <iostream>

gams::algorithms::Printer_Algorithm::Printer_Algorithm (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base (knowledge, platform, sensors, self)
{
  status_.init_vars (*knowledge, "debug");
}

gams::algorithms::Printer_Algorithm::~Printer_Algorithm ()
{
}

void
gams::algorithms::Printer_Algorithm::operator= (const Printer_Algorithm & rhs)
{
  if (this != &rhs)
  {
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
  }
}


int
gams::algorithms::Printer_Algorithm::analyze (void)
{
  std::cerr << "algorithm.analyze ()" << std::endl;

  if (platform_)
    std::cerr << "  algorithm.platform_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.platform is not set." << std::endl;
  
  if (sensors_)
    std::cerr << "  algorithm.sensors_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.sensors_ is not set." << std::endl;
  
  if (self_)
    std::cerr << "  algorithm.self_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.self_ is not set." << std::endl;
  
  status_.waiting = 1;
  status_.ok = 1;

  std::cerr << "  algorithm.status_.ok == " << *status_.ok << std::endl;
  std::cerr << "  algorithm.status_.paused == " << *status_.paused << std::endl;
  std::cerr << "  algorithm.status_.waiting == " << *status_.waiting << std::endl;
  std::cerr << "  algorithm.status_.deadlocked == "
    << *status_.deadlocked << std::endl;
  std::cerr << "  algorithm.status_.failed == " << *status_.failed << std::endl;
  std::cerr << "  algorithm.status_.unknown == " << *status_.unknown << std::endl;

  return 0;
}
      

int
gams::algorithms::Printer_Algorithm::execute (void)
{
  std::cerr << "algorithm.execute ()" << std::endl;

  if (platform_)
    std::cerr << "  algorithm.platform_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.platform is not set." << std::endl;
  
  if (sensors_)
    std::cerr << "  algorithm.sensors_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.sensors_ is not set." << std::endl;
  
  if (self_)
    std::cerr << "  algorithm.self_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.self_ is not set." << std::endl;
  
  status_.waiting = 0;

  std::cerr << "  algorithm.status_.ok == " << *status_.ok << std::endl;
  std::cerr << "  algorithm.status_.paused == " << *status_.paused << std::endl;
  std::cerr << "  algorithm.status_.waiting == " << *status_.waiting << std::endl;
  std::cerr << "  algorithm.status_.deadlocked == "
    << *status_.deadlocked << std::endl;
  std::cerr << "  algorithm.status_.failed == " << *status_.failed << std::endl;
  std::cerr << "  algorithm.status_.unknown == " << *status_.unknown << std::endl;

  if (platform_)
  {
    utility::GPS_Position next (1, 2, 3);
    platform_->move (next);
  }
  else
  {
    std::cerr << "  ERROR: platform_ is null. Cannot call move ()." << std::endl;
  }

  return 0;
}


int
gams::algorithms::Printer_Algorithm::plan (void)
{
  std::cerr << "algorithm.plan ()" << std::endl;

  if (platform_)
    std::cerr << "  algorithm.platform_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.platform is not set." << std::endl;
  
  if (sensors_)
    std::cerr << "  algorithm.sensors_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.sensors_ is not set." << std::endl;
  
  if (self_)
    std::cerr << "  algorithm.self_ is set." << std::endl;
  else
    std::cerr << "  ERROR: algorithm.self_ is not set." << std::endl;

  std::cerr << "  algorithm.status_.ok == " << *status_.ok << std::endl;
  std::cerr << "  algorithm.status_.paused == " << *status_.paused << std::endl;
  std::cerr << "  algorithm.status_.waiting == " << *status_.waiting << std::endl;
  std::cerr << "  algorithm.status_.deadlocked == "
    << *status_.deadlocked << std::endl;
  std::cerr << "  algorithm.status_.failed == " << *status_.failed << std::endl;
  std::cerr << "  algorithm.status_.unknown == " << *status_.unknown << std::endl;

  return 0;
}
