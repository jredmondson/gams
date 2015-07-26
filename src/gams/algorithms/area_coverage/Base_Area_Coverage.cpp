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
 * Base_Area_Coverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file defines common functionality for area coverage algorithms. If this
 * file is updated, please also update the tutorials in the wiki with line
 * numbers or implementation changes.
 */

#include "gams/algorithms/area_coverage/Base_Area_Coverage.h"

gams::algorithms::area_coverage::Base_Area_Coverage::Base_Area_Coverage (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices,
  const ACE_Time_Value& e_time) :
  Base_Algorithm (knowledge, platform, sensors, self, devices), 
  exec_time_ (e_time), end_time_(ACE_OS::gettimeofday () + e_time)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::Base_Area_Coverage:" \
    " constructor succeeded\n");
}

gams::algorithms::area_coverage::Base_Area_Coverage::~Base_Area_Coverage ()
{
}

void
gams::algorithms::area_coverage::Base_Area_Coverage::operator= (
  const Base_Area_Coverage & rhs)
{
  if (this != &rhs)
  {
    this->next_position_ = rhs.next_position_;
    this->exec_time_ = rhs.exec_time_;
    this->end_time_ = rhs.end_time_;

    this->Base_Algorithm::operator= (rhs);
  }
}

/**
 * Many of the area coverage algorithms need only increment the execution
 * counter in their analyze phase. Algorithms using sensors will override this
 * with sensor analysis.
 */
int
gams::algorithms::area_coverage::Base_Area_Coverage::analyze ()
{
  ++executions_;
  int ret_val = check_if_finished (OK);
  if (ret_val == FINISHED)
    status_.finished = 1;
  return check_if_finished (OK);
}

/**
 * All of the area coverage algorithms have simple execution steps of just
 * moving to their destination.
 */
int
gams::algorithms::area_coverage::Base_Area_Coverage::execute ()
{
  if (status_.finished != 1)
    platform_->move(next_position_);
  return 0;
}

/**
 * All of the area coverage algorithms just check if they have reached close
 * enough to their destination. If so, they generate a new destination.
 */
int
gams::algorithms::area_coverage::Base_Area_Coverage::plan ()
{
  // generate new next position if necessary
  double dist = platform_->get_position()->distance_to(next_position_);
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::Base_Area_Coverage:" \
    " distance between points is %f\n", dist);
  
  if (platform_->get_position()->approximately_equal(next_position_, platform_->get_accuracy()))
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::area_coverage::Base_Area_Coverage:" \
      " generating new position\n");
    generate_new_position();
  }

  return 0;
}

gams::utility::GPS_Position
gams::algorithms::area_coverage::Base_Area_Coverage::get_next_position() const
{
  return next_position_;
}

int
gams::algorithms::area_coverage::Base_Area_Coverage::check_if_finished (
  int ret_val) const
{
  if (exec_time_ != 0 && ret_val == OK && (ACE_OS::gettimeofday () > end_time_))
    ret_val = FINISHED;
  return ret_val;
}
