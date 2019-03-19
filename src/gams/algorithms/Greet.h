/**
 * Copyright(c) 2019 James Edmondson. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and any disclaimers in the documentation
 *    and/or other materials provided with the distribution.
 **/

/**
 * @file Greet.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the declaration of the Greet algorithm. This algorithm
 * is meant to created a decentralized process for moving agents toward a target
 * and potentially following them once the target has passed a point of interest
 * to the agent.
 **/

#ifndef   _GAMS_ALGORITHMS_GREET_H_
#define   _GAMS_ALGORITHMS_GREET_H_

#include <memory>

#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/pose/CartesianFrame.h"
#include "gams/groups/GroupBase.h"
#include "gams/groups/GroupFactoryRepository.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for greeting a target if it comes within range and potentially
    * following the target around.
    **/
    class GAMS_EXPORT Greet : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  target           full agent name the agent should follow
       * @param  target_group     group name for agents in the target group 
       * @param  guard_distance   distance in meters to guard from guard
       *                          guard location
       * @param  guard_location   location to consider guard territory for
       *                          this agent
       * @param  guard_max_follow_distance distance that a guard can reach
       *                          from its guard location before it returns
       *                          back to its area. During following, this
       *                          can prevent infinite following. -1 means
       *                          do not rubberband but follow indefinitely
       * @param  home_location    location to perch or home at if not
       *                          guarding or following
       * @param  follow           if true, follow up to follow_max_agents in
       *                          the specified follow group
       * @param  follow_group     group name for agents following a target.
       *                          This is a special group name that will
       *                          allow expansion if you use the .target
       *                          variable in the name. By default this,
       *                          variable is group.{.target}.follow, which
       *                          would expand to something like:
       *                          group.agent.0.follow for agents following
       *                          agent.0
       * @param  follow_max_agents maximum agents to follow a target at any
       *                          instantaneous time
       * @param  knowledge  the context containing variables and values
       * @param  platform   the underlying platform the algorithm will use
       * @param  sensors    map of sensor names to sensor information
       * @param  self       self-referencing variables
       **/
      Greet(
        const std::string & target, const std::string & target_group,
        double guard_distance,
        const std::vector<double> & guard_location,
        double guard_max_follow_distance,
        const std::vector<double> & home_location,
        bool follow, const std::string & follow_group,
        int follow_max_agents,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);
      
      /**
       * Destructor
       **/
      ~Greet();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator=(const Greet & rhs);
      
      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze(void);
      
      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute(void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan(void);
      
    protected:
      /// location of agent to follow
      variables::Agent target_;

      /// a group of targets to watch
      std::unique_ptr<groups::GroupBase> target_group_ = 0;

      /// if true, indicates to follow targets that enter guard area
      bool follow_ = false;

      /// a group of agents following to watch
      std::unique_ptr<groups::GroupBase> follow_group_ = 0;

      /// name of the group that is following target
      std::string follow_group_name_ = "group.{.target}.followers";

      /// the group name prefix before {.target}
      std::string follow_group_prefix_;

      /// the group name suffix after {.target}
      std::string follow_group_suffix_;

      /// maximum number of agents that should follow a target
      int follow_max_agents_ = 2;

      /// maximum follow distance from guard location
      double follow_max_distance_ = 1000;

      /// maximum distance a guard can be displaced from guard location before
      /// it returns to its guard area (prevents infinite follow)
      double guard_max_follow_distance_ = -1;

      /// the distance from guard location (max of dimensions)
      pose::Epsilon guard_epsilon_;

      /// the location for the agent to guard
      pose::Position guard_location_;

      // factory for interacting with user-defined groups
      groups::GroupFactoryRepository group_factory_;

      /// the home location to return to if not guarding or following
      pose::Position home_location_;

      /// indicates whether or not the agent is following a target
      bool is_following_ = false;

      /// indicates whether or not the agent is moving to guard
      bool is_guarding_ = false;

      /// if no custom home location is specified, use the agent's home
      bool use_agent_home_ = true;
    };

    /**
     * A factory class for creating Greet Algorithms
     **/
    class GAMS_EXPORT GreetFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Greet Algorithm.
       * @param   args      target = the target to follow
       *                    delay = the time step delay
       * @param   knowledge the knowledge base to use
       * @param   platform  the platform. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       * @param   agents   the list of agents, which is dictated by
       *                    init_vars when a number of processes is set. This
       *                    will be set by the controller in init_vars
       **/
      virtual BaseAlgorithm * create(
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        platforms::BasePlatform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Agents * agents);
    };
  }
}

#endif // _GAMS_ALGORITHMS_GREET_H_
