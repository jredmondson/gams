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

/**
 * @file Message_Profiling.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the declaration of the Message_Profiling Algorithm for 
 * testing messaging performance between peers. 
 **/

#ifndef _GAMS_ALGORITHMS_MESSAGE_PROFILING_H_
#define _GAMS_ALGORITHMS_MESSAGE_PROFILING_H_

#include "gams/algorithms/Base_Algorithm.h"

#include <map>
#include <vector>
#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/variables/Algorithm.h"
#include "gams/variables/Self.h"

#include "madara/filters/Aggregate_Filter.h"

namespace gams
{
  namespace algorithms
  {
    class GAMS_Export Message_Profiling : public Base
    {
    public:
      /**
       * Constructor
       * @param  send         size of data to send, 0 to not send
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       **/
      Message_Profiling (
        size_t send,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
        platforms::Base * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~Message_Profiling ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Message_Profiling & rhs);
      
      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void);
      
      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute (void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan (void);

    private:
      /**
       * Prefix for message keys
       */
      const static std::string key_prefix_;

      /**
       * Filter for tracking which messages have come in and which have been 
       * dropped
       */
      class Message_Filter : public Madara::Filters::Aggregate_Filter
      {
      public:
        /**
         * virtual destructor
         */
        virtual ~Message_Filter ();

        void filter (Madara::Knowledge_Map& records, 
          const Madara::Transport::Transport_Context& transport_context,
          Madara::Knowledge_Engine::Variables& var);

        std::string missing_messages_string () const;

      private:
        /**
         * Message_Data struct
         */
        struct Message_Data
        {
          size_t first;
          size_t last;
          std::vector<bool> present;

          Message_Data ();
        };

        /**
         * Keep a Message_Data struct for each peer
         */
        std::map<std::string, Message_Data> msg_map_;
      };

      /**
       * Message Filter object
       */
      Message_Filter filter_;

      /**
       * size of message to send 
       */
      size_t send_size_;

    };

    /**
     * A factory class for creating Message_Profiling Algorithms
     **/
    class GAMS_Export Message_Profiling_Factory : public Algorithm_Factory
    {
    public:

      /**
       * Creates a Message_Profiling Algorithm.
       * @param   args      arg[0] = the message send size
       * @param   platform  the platform. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       * @param   devices   the list of devices, which is dictated by
       *                    init_vars when a number of processes is set. This
       *                    will be set by the controller in init_vars
       **/
      virtual Base_Algorithm * create (
        const Madara::Knowledge_Vector & args,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        platforms::Base_Platform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Devices * devices);
    };
  }
}

#endif // _GAMS_ALGORITHMS_MESSAGE_PROFILING_H_
