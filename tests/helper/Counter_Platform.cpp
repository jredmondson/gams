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
#include "Counter_Platform.h"
#include "gams/utility/Logging.h"


gams::platforms::Counter_Platform::Counter_Platform (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge)
  : Base_Platform (&knowledge)
{
  analyze_counter_.set_name (".platform_analyzes", knowledge);
  get_gps_accuracy_counter_.set_name (".platform_get_gps_accuracies", knowledge);
  get_move_speed_counter_.set_name (".platform_get_move_speeds", knowledge);
  home_counter_.set_name (".platform_homes", knowledge);
  land_counter_.set_name (".platform_lands", knowledge);
  move_counter_.set_name (".platform_moves", knowledge);
  sense_counter_.set_name (".platform_senses", knowledge);
  set_move_speed_counter_.set_name (".platform_set_move_speeds", knowledge);
  takeoff_counter_.set_name (".platform_takeoffs", knowledge);
}

gams::platforms::Counter_Platform::~Counter_Platform ()
{
}

void
gams::platforms::Counter_Platform::operator= (const Counter_Platform & rhs)
{
  if (this != &rhs)
  {
    platforms::Base_Platform * dest = dynamic_cast <platforms::Base_Platform *> (this);
    const platforms::Base_Platform * source =
      dynamic_cast <const platforms::Base_Platform *> (&rhs);

    *dest = *source;
    this->analyze_counter_ = rhs.analyze_counter_;
    this->get_gps_accuracy_counter_ = rhs.get_gps_accuracy_counter_;
    this->get_move_speed_counter_ = rhs.get_move_speed_counter_;
    this->home_counter_ = rhs.home_counter_;
    this->land_counter_ = rhs.land_counter_;
    this->move_counter_ = rhs.move_counter_;
    this->sense_counter_ = rhs.sense_counter_;
    this->set_move_speed_counter_ = rhs.set_move_speed_counter_;
    this->takeoff_counter_ = rhs.takeoff_counter_;
  }
}
 
int
gams::platforms::Counter_Platform::analyze (void)
{
  //++analyze_counter_;

  return 0;
}

std::string
gams::platforms::Counter_Platform::get_id () const
{
  return "counter_platform";
}

std::string
gams::platforms::Counter_Platform::get_name () const
{
  return "Counter Platform";
}

double
gams::platforms::Counter_Platform::get_accuracy () const
{
  //++get_gps_accuracy_counter_;
  
  return 0.0;
}

double
gams::platforms::Counter_Platform::get_move_speed () const
{
  //++get_move_speed_counter_;
  return 0.0;
}

int
gams::platforms::Counter_Platform::home (void)
{
  //++home_counter_;
  
  return 0;
}

int
gams::platforms::Counter_Platform::land (void)
{
  //++land_counter_;
  
  return 0;
}

int
gams::platforms::Counter_Platform::move (const utility::Position & position,
  const double & /*epsilon*/)
{
  //++move_counter_;
  
  return 0;
}

int
gams::platforms::Counter_Platform::sense (void)
{
  //++sense_counter_;
  
  return 0;
}

void
gams::platforms::Counter_Platform::set_move_speed (const double& /*speed*/)
{
  //++set_move_speed_counter_;
}

int
gams::platforms::Counter_Platform::takeoff (void)
{
  //++takeoff_counter_;
  
  return 0;
}
