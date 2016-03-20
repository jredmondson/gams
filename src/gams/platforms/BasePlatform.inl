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

inline gams::platforms::BasePlatform::BasePlatform (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
  : knowledge_ (knowledge), self_ (self), sensors_ (sensors)
{
}

inline gams::utility::Position *
gams::platforms::BasePlatform::get_position ()
{
  utility::Position * position = new utility::Position ();
  position->from_container (self_->agent.location);
  return position;
}

inline gams::utility::Location
gams::platforms::BasePlatform::get_location () const
{
  utility::Location ret(get_frame(), 0, 0);
  ret.from_container<utility::order::GPS>(self_->agent.location);
  return ret;
}

inline gams::utility::Rotation
gams::platforms::BasePlatform::get_rotation () const
{
  return utility::Rotation(get_frame(), self_->agent.angle);
}

inline gams::utility::Pose
gams::platforms::BasePlatform::get_pose () const
{
  return utility::Pose(get_frame(), self_->agent.location,
                       self_->agent.angle);
}

inline madara::knowledge::KnowledgeBase *
gams::platforms::BasePlatform::get_knowledge_base (void) const
{
  return knowledge_;
}

inline gams::variables::Self *
gams::platforms::BasePlatform::get_self (void) const
{
  return self_;
}

inline gams::variables::Sensors *
gams::platforms::BasePlatform::get_sensors (void) const
{
  return sensors_;
}

inline const gams::variables::PlatformStatus *
gams::platforms::BasePlatform::get_platform_status (void) const
{
  return &status_;
}

inline gams::variables::PlatformStatus *
gams::platforms::BasePlatform::get_platform_status (void)
{
  return &status_;
}

inline const gams::utility::ReferenceFrame &
gams::platforms::BasePlatform::get_frame (void)
{
  return frame_;
}
