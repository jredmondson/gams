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
#include "CounterPlatform.h"
#include "gams/loggers/GlobalLogger.h"


gams::platforms::CounterPlatform::CounterPlatform (
  madara::knowledge::KnowledgeBase & knowledge)
  : BasePlatform (&knowledge)
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

gams::platforms::CounterPlatform::~CounterPlatform ()
{
}

void
gams::platforms::CounterPlatform::operator= (const CounterPlatform & rhs)
{
  if (this != &rhs)
  {
    platforms::BasePlatform * dest = dynamic_cast <platforms::BasePlatform *> (this);
    const platforms::BasePlatform * source =
      dynamic_cast <const platforms::BasePlatform *> (&rhs);

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
gams::platforms::CounterPlatform::analyze (void)
{
  //++analyze_counter_;

  return 0;
}

std::string
gams::platforms::CounterPlatform::get_id () const
{
  return "counter_platform";
}

std::string
gams::platforms::CounterPlatform::get_name () const
{
  return "Counter Platform";
}

double
gams::platforms::CounterPlatform::get_accuracy () const
{
  //++get_gps_accuracy_counter_;
  
  return 0.0;
}

double
gams::platforms::CounterPlatform::get_move_speed () const
{
  //++get_move_speed_counter_;
  return 0.0;
}

int
gams::platforms::CounterPlatform::home (void)
{
  //++home_counter_;
  
  return 0;
}

int
gams::platforms::CounterPlatform::land (void)
{
  //++land_counter_;
  
  return 0;
}

int
gams::platforms::CounterPlatform::move (const utility::Position & /*position*/,
  const double & /*epsilon*/)
{
  //++move_counter_;
  
  return 0;
}

int
gams::platforms::CounterPlatform::sense (void)
{
  //++sense_counter_;
  
  return 0;
}

void
gams::platforms::CounterPlatform::set_move_speed (const double& /*speed*/)
{
  //++set_move_speed_counter_;
}

int
gams::platforms::CounterPlatform::takeoff (void)
{
  //++takeoff_counter_;
  
  return 0;
}
