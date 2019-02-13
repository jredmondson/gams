/**
 * Copyright (c) 2017 Carnegie Mellon University. All Rights Reserved.
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
 * @file ControllerSettings.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains controller settings for controller initializations
 **/

#ifndef   _GAMS_CONTROLLERS_CONTROLLERSETTINGS_H_
#define   _GAMS_CONTROLLERS_CONTROLLERSETTINGS_H_

#include <string>

#include "madara/knowledge/EvalSettings.h"
#include "gams/GamsExport.h"

namespace gams { namespace controllers {

enum CheckpointStrategies
{
  CHECKPOINT_NONE = 0,
  CHECKPOINT_EVERY_LOOP = 1,
  CHECKPOINT_EVERY_SEND = 2,
  CHECKPOINT_SAVE_DIFFS = 4,
  CHECKPOINT_SAVE_FULL_CONTEXTS = 8,
  CHECKPOINT_SAVE_ONE_FILE = 16,
  CHECKPOINT_SAVE_DIFFS_IN_ONE_FILE = 20,
  CHECKPOINT_STREAM_TO_FILE = 32
};

enum ThreadingStrategies
{
  THREADS_NONE = 0,
  THREADS_ONE_PER_CONTROLLER = 1
};

enum SchedulingStrategies
{
  SCHEDULE_ROUND_ROBIN = 0,
  SCHEDULE_UNIFORM_RANDOM = 1,
  SCHEDULE_FAIR_RANDOM = 2
};

/**
 * Settings used for initializing GAMS controllers
 **/
class GAMS_EXPORT ControllerSettings
{
public:
  /**
   * Constructor
   **/
  ControllerSettings () = default;

  /**
   * the default agent prefix (e.g., "agent.bob" or "agent.0"). This is a prefix
   * of what the self_ agent prefix will be in the knowledge base. For instance,
   * agent.0.location, agent.0.algorithm, etc.
   **/
  std::string agent_prefix = "agent.0";

  /**
   * the knowledge checkpointing file system prefix (e.g., "./checkpoint" will
   * save checkpoints to currently directory in files that start with checkpoint
   **/
  std::string checkpoint_prefix = "checkpoint";

  /// the knowledge checkpointing strategy
  int checkpoint_strategy = CHECKPOINT_NONE;

  /// the gams logging level (negative means don't change)
  int gams_log_level = -1;

  /// the hertz rate that a controller should run at
  double loop_hertz = 2.0;

  /// the MADARA logging level (negative means don't change)
  int madara_log_level = -1;

  /// maximum runtime (-1 means persistent, forever)
  double run_time = -1;

  /// the hertz rate to call send_modifieds at
  double send_hertz = 1.0;

  /// settings used in any KnowledgeBase operations by this controller
  madara::knowledge::EvalSettings eval_settings =
    madara::knowledge::EvalSettings::SEND;

  /// for multicontrollers, launch each controller in a separate thread
  int threading_strategy = THREADS_NONE;

  /// for multicontrollers, call run on controllers in a round robin way
  int scheduling_strategy = SCHEDULE_ROUND_ROBIN;

  /// include a shared memory transport when managing multiple controllers
  bool shared_memory_transport = true;
};

} }

#endif // _GAMS_CONTROLLERS_CONTROLLERSETTINGS_H_
