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

#include "gams/algorithms/Base_Algorithm.h"
#include "gams/utility/Region.h"

namespace variables = gams::variables;
namespace platforms = gams::platforms;

gams::algorithms::Base::Base (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
  : devices_ (devices), executions_ (0), knowledge_ (knowledge),
    platform_ (platform), self_ (self), sensors_ (sensors)
{
}

gams::algorithms::Base::~Base ()
{
}

void
gams::algorithms::Base::operator= (const Base & rhs)
{
  if (this != &rhs)
  {
    this->knowledge_ = rhs.knowledge_;
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
  }
}

void
gams::algorithms::Base::set_devices (variables::Devices * devices)
{
  devices_ = devices;
}

void
gams::algorithms::Base::set_platform (platforms::Base * platform)
{
  platform_ = platform;
}

void
gams::algorithms::Base::set_self (variables::Self * self)
{
  self_ = self;
}

void
gams::algorithms::Base::set_sensors (variables::Sensors * sensors)
{
  sensors_ = sensors;
}

variables::Devices *
gams::algorithms::Base::get_devices (void)
{
  return devices_;
}

Madara::Knowledge_Engine::Knowledge_Base *
gams::algorithms::Base::get_knowledge_base (void)
{
  return knowledge_;
}

platforms::Base *
gams::algorithms::Base::get_platform (void)
{
  return platform_;
}

variables::Self *
gams::algorithms::Base::get_self (void)
{
  return self_;
}

variables::Sensors *
gams::algorithms::Base::get_sensors (void)
{
  return sensors_;
}

variables::Algorithm *
gams::algorithms::Base::get_algorithm_status (void)
{
  return &status_;
}
