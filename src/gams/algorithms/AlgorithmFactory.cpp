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

#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/algorithms/Land.h"
#include "gams/algorithms/Move.h"
#include "gams/algorithms/DebugAlgorithm.h"
#include "gams/algorithms/NullAlgorithm.h"
#include "gams/algorithms/FormationFlying.h"
#include "gams/algorithms/FormationCoverage.h"
#include "gams/algorithms/Takeoff.h"
#include "gams/algorithms/Follow.h"
#include "gams/algorithms/MessageProfiling.h"

#include "gams/algorithms/area_coverage/UniformRandomAreaCoverage.h"
#include "gams/algorithms/area_coverage/UniformRandomEdgeCoverage.h"
#include "gams/algorithms/area_coverage/PriorityWeightedRandomAreaCoverage.h"
#include "gams/algorithms/area_coverage/LocalPheremoneAreaCoverage.h"
#include "gams/algorithms/area_coverage/SnakeAreaCoverage.h"
#include "gams/algorithms/area_coverage/MinTimeAreaCoverage.h"
#include "gams/algorithms/area_coverage/PrioritizedMinTimeAreaCoverage.h"
#include "gams/algorithms/area_coverage/PerimeterPatrol.h"

#include <iostream>

using std::cerr;
using std::endl;

gams::algorithms::AlgorithmFactory::AlgorithmFactory ()
  : knowledge_ (0), devices_ (0), platform_ (0), self_ (0), sensors_ (0)
{
}

gams::algorithms::AlgorithmFactory::~AlgorithmFactory ()
{
}

void
gams::algorithms::AlgorithmFactory::set_devices (variables::Devices * devices)
{
  devices_ = devices;
}

void
gams::algorithms::AlgorithmFactory::set_knowledge (
  madara::knowledge::KnowledgeBase * knowledge)
{
  knowledge_ = knowledge;
}

void
gams::algorithms::AlgorithmFactory::set_platform (platforms::BasePlatform * platform)
{
  platform_ = platform;
}

void
gams::algorithms::AlgorithmFactory::set_self (variables::Self * self)
{
  self_ = self;
}

void
gams::algorithms::AlgorithmFactory::set_sensors (variables::Sensors * sensors)
{
  sensors_ = sensors;
}
