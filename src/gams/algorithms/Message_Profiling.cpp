/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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

#include "gams/algorithms/Message_Profiling.h"

#include <iostream>
#include <sstream>

using std::stringstream;
using std::string;
using std::map;
using std::cerr;
using std::endl;

gams::algorithms::Message_Profiling::Message_Profiling (size_t send, 
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base (knowledge, platform, sensors, self), send_size_(send)
{
  status_.init_vars (*knowledge, "message_profiling");

  // attach filter
  knowledge->close_transport ();

  Madara::Transport::QoS_Transport_Settings& settings = 
    dynamic_cast<Madara::Transport::QoS_Transport_Settings&>(
      knowledge->transport_settings ());
  settings.add_receive_filter (Madara::Knowledge_Record::STRING, 
    check_messages);
  settings.set_send_bandwidth_limit (-1);
  settings.set_total_bandwidth_limit (-1);

  knowledge->attach_transport (knowledge->get_id (), settings);
}

gams::algorithms::Message_Profiling::~Message_Profiling ()
{
}

void
gams::algorithms::Message_Profiling::operator= (
  const Message_Profiling & rhs)
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
gams::algorithms::Message_Profiling::analyze (void)
{
  
  return 0;
}
      

int
gams::algorithms::Message_Profiling::execute (void)
{
  ++executions_;

  if (send_size_ != 0)
  {
    // key: "message_profiler.{.id}.data"
    // value: "<counter>," + arbitrary data of size send_size_
    string key = "message_profiler." + knowledge_->get (".id").to_string () + 
      ".data";
    stringstream value;
    value << executions_ << ",";
    string value_str = value.str ();
    if(value_str.length () < send_size_) // fill to size of send_size_
      value_str = value_str + string (send_size_ - value_str.length (), 'a');
  
    // actually set knowledge
    knowledge_->set (key, value_str);
  }

  return 0;
}


int
gams::algorithms::Message_Profiling::plan (void)
{
  return 0;
}

Madara::Knowledge_Record
gams::algorithms::Message_Profiling::check_messages (
  Madara::Knowledge_Engine::Function_Arguments & args, 
  Madara::Knowledge_Engine::Variables & variables)
{
  // get id from "message_profiler.{.id}.data
  string check;
  string id;
  stringstream key (args[1].to_string ());
  std::getline (key, check, '.');
  std::getline (key, id, '.');

  if (check == "message_profiler")
  {
    string counter_str;
    key.clear ();
    stringstream value (args[0].to_string());
    std::getline (value, counter_str, ',');
    value.str (counter_str);
    size_t counter;
    value >> counter;
  
    // check with current info
    string key_base = ".message_profiler." + id + ".counters";
    Madara::Knowledge_Record first = variables.get (key_base + ".first_message");
    Madara::Knowledge_Record last = variables.get (key_base + ".last_message");
    Madara::Knowledge_Record dropped = variables.get (
      key_base + ".dropped_counter");
    if (first.to_integer () == 0)
    {
      first = Madara::Knowledge_Record::Integer (counter);
      last = Madara::Knowledge_Record::Integer (counter);
      dropped = Madara::Knowledge_Record::Integer (0);
    }
    else
    {
      if (counter > last.to_integer () + 1)
        dropped += Madara::Knowledge_Record::Integer (
          counter - (last.to_integer () + 1));
      last = Madara::Knowledge_Record::Integer (counter);
    }
  
    // set updates
    variables.set (key_base + ".first_message", first);
    variables.set (key_base + ".last_message", last);
    variables.set (key_base + ".dropped_counter", dropped);
  }
  
  // we don't really care what is returned, all the interesting processing has 
  // already been done by the time we reach here
  return args[0];
}
