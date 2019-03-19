/**
 * Copyright(c) 2016 Carnegie Mellon University. All Rights Reserved.
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
 * @file VREPBase.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the inline functions of the VREPBase class
 **/

#ifndef   _GAMS_PLATFORM_VREP_BASE_INL_
#define   _GAMS_PLATFORM_VREP_BASE_INL_

#include "VREPBase.h"

#ifdef _GAMS_VREP_

inline bool
gams::platforms::VREPBase::sim_is_running(void)
{
  if (!sim_is_running_)
  {
    sim_is_running_ = begin_sim_ != 0;
  }

  return sim_is_running_;
}

inline bool
gams::platforms::VREPBase::vrep_is_ready(void)
{
  if (!vrep_is_ready_)
  {
    vrep_is_ready_ = vrep_ready_ != 0;
  }

  return vrep_is_ready_;
}

inline bool
gams::platforms::VREPBase::agent_is_ready(void)
{
  if (!agent_is_ready_)
  {
    agent_is_ready_ = agent_ready_ != 0;
  }

  return agent_is_ready_;
}

inline bool
gams::platforms::VREPBase::get_ready(void)
{
  bool result(sim_is_running());

  if (!result && this->vrep_is_ready())
  {
    if (!agent_is_ready())
    {
      add_model_to_environment(model_file_, is_client_side_);
      set_initial_position();
      get_target_handle();

      agent_ready_ = 1;
    }

    result = this->sim_is_running();
  }

  if (result)
  {
    status_.ok = 1;
  }
  else
  {
    status_.ok = 0;
  }

  return result;
}

#endif // _GAMS_VREP_

#endif // _GAMS_PLATFORM_VREP_BASE_INL_
