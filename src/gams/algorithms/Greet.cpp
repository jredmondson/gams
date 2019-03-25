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
 * @file Greet.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the Greet algorithm. This algorithm
 * is meant to created a decentralized process for moving agents toward a target
 * and potentially following them once the target has passed a point of interest
 * to the agent.
 */

#include "gams/algorithms/Greet.h"

#include <sstream>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>

#include "madara/utility/Timer.h"
#include "gams/utility/ArgumentParser.h"
#include "gams/auctions/AuctionMinimumDistance.h"

using std::stringstream;

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;
typedef knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;
typedef madara::utility::Timer<madara::utility::Clock> Timer;

gams::algorithms::BaseAlgorithm *
gams::algorithms::GreetFactory::create(
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result(0);

  // set defaults
  std::string target;
  std::string target_group = "group.targets";
  std::string guard_group = ".group.guards";
  double guard_distance = 20.0;
  std::vector <double> guard_location;
  double guard_max_follow_distance = -1;
  std::vector <double> home_location;
  bool follow = false;
  std::string follow_group = "group.{.target}.follow";
  Integer follow_max_agents = 2;
  

  if (knowledge && platform && self)
  {
    for (KnowledgeMap::const_iterator i = args.begin(); i != args.end(); ++i)
    {
      if (i->first.size() <= 0)
        continue;

      switch (i->first[0])
      {
      case 'f':
        if (i->first == "follow.group")
        {
          follow_group = i->second.to_string();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " members for follow found in group %s\n", target_group.c_str());
          follow = true;

          break;
        }
        else if (i->first == "follow")
        {
          follow = i->second.is_true();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " setting follow to %s\n", follow ? "true" : "false");

          break;
        }
        else if (i->first == "follow.max_agents" ||
                i->first == "follow.max.agents" ||
                i->first == "follow.agents.max")
        {
          follow_max_agents = (int)i->second.to_integer();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " setting follow max agents to %d\n", follow_max_agents);
          follow = true;

          break;
        }
        goto unknown;
      case 'g':
        if (i->first == "guard.location")
        {
          guard_location = i->second.to_doubles();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " guard location set to [%s]\n",  i->second.to_string().c_str());

          break;
        }
        else if (i->first == "guard.distance")
        {
          guard_distance = i->second.to_double();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " guard distance set to %f\n", guard_distance);

          break;
        }
        else if (i->first == "guard.group")
        {
          guard_group = i->second.to_string();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " guard group set to %s\n", guard_group.c_str());

          break;
        }
        else if (i->first == "guard.max_follow_distance")
        {
          guard_max_follow_distance = i->second.to_double();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " guard max follow distance set to %f\n",
            guard_max_follow_distance);

          break;
        }
        goto unknown;
      case 'h':
        if (i->first == "home.location")
        {
          home_location = i->second.to_doubles();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " home location set to [%s]\n",  i->second.to_string().c_str());

          break;
        }
        goto unknown;
      case 't':
        if (i->first == "target.group")
        {
          target_group = i->second.to_string();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::GreetFactory:" \
            " setting formation head/target to group %s\n",
            target_group.c_str());

          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::GreetFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str(), i->second.to_string().c_str());

        break;
      } // end switch
    } // end iterate over args

    // if group has not been set, use the swarm
    if (target_group == "")
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::GreetFactory::create:" \
        " No target specified. Returning null.\n");
    }
    else
    {
      result = new Greet(
        target, target_group,
        guard_distance, guard_group, guard_location, guard_max_follow_distance,
        home_location,
        follow, follow_group, follow_max_agents,
        knowledge, platform, sensors, self);
    }
  }

  return result;
}

gams::algorithms::Greet::Greet(
  const std::string & target, const std::string & target_group,
  double guard_distance,
  const std::string & guard_group,
  const std::vector<double> & guard_location,
  double guard_max_follow_distance,
  const std::vector<double> & home_location,
  bool follow, const std::string & follow_group, int follow_max_agents,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm(knowledge, platform, sensors, self),
  group_factory_ (knowledge),
  guard_epsilon_(guard_distance),
  guard_location_(platform->get_frame()),
  guard_max_follow_distance_ (guard_max_follow_distance),
  home_location_(platform->get_frame())
{
  if (knowledge && platform && sensors && self)
  {
    status_.init_vars(*knowledge, "greeting", self->agent.prefix);
    status_.init_variable_values();

    if (target != "")
    {
      // initialize leader's variables
      target_.init_vars(*knowledge, target);
    }

    if (guard_group != "")
    {
      guard_group_.reset(
        group_factory_.create(guard_group));
    }

    if (target_group != "")
    {
      target_group_.reset(
        group_factory_.create(target_group));
    }

    if (home_location.size() >= 2)
    {
      home_location_.x(home_location[0]);
      home_location_.y(home_location[1]);

      if (home_location.size() >= 3)
      {
        home_location_.z(home_location[2]);
      }

      use_agent_home_ = false;
    } // end if custom home is set
    else
    {
      // if no custom home, use the {.prefix}.home
      use_agent_home_ = true;
    }
    

    if (guard_location.size() >= 2)
    {
      guard_location_.x(guard_location[0]);
      guard_location_.y(guard_location[1]);

      if (guard_location.size() >= 3)
      {
        guard_location_.z(guard_location[2]);
      }
    }

    follow_ = follow;
    follow_max_agents_ = follow_max_agents;

    // note that follow group is a bit weird. If it has .target in
    // it, then it is a dynamic follow group that is specified per
    // target. So, it's something we need to keep track of with
    // every iteration of analyze() not in the constructor
    if (follow_group != "")
    {
      follow_group_name_ = follow_group;
    }

    size_t pattern_found = follow_group_name_.find("{.target}");

    if (pattern_found != std::string::npos)
    {
      follow_group_prefix_ = follow_group_name_.substr(0, pattern_found);
      follow_group_suffix_ = follow_group_name_.substr(
        pattern_found + sizeof("{.target}") - 1);
    }

    if (follow_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Greet::ctr:"
        " agent configured to guard and follow.\n");
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Greet::ctr:"
        " agent configured to guard only.\n");
    }
  }
}

gams::algorithms::Greet::~Greet()
{
}

void
gams::algorithms::Greet::operator=(const Greet & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator=(rhs);
    this->target_ = rhs.target_;
    this->follow_ = rhs.follow_;
    this->follow_group_name_ = rhs.follow_group_name_;
    this->follow_group_prefix_ = rhs.follow_group_prefix_;
    this->follow_group_suffix_ = rhs.follow_group_suffix_;
    this->follow_max_agents_ = rhs.follow_max_agents_;
    this->guard_epsilon_ = rhs.guard_epsilon_;
    this->guard_location_ = rhs.guard_location_;
    this->home_location_ = rhs.home_location_;
    this->is_following_ = rhs.is_following_;
    this->is_guarding_ = rhs.is_guarding_;
    this->use_agent_home_ = rhs.use_agent_home_;

    if (rhs.target_group_.get() != 0)
    {
      this->target_group_.reset(
        group_factory_.create(rhs.target_group_->get_prefix()));
    }

    if (rhs.follow_group_.get() != 0)
    {
      this->follow_group_ .reset(
        group_factory_.create(rhs.follow_group_->get_prefix()));
    }
  }
}

/**
 * The agent gets the target's location from the database and adds it to the
 * queue of positions being stored.
 */
int
gams::algorithms::Greet::analyze(void)
{
  if (self_)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Greet::analyze:" \
      " current pose is [%s, %s].\n",
      self_->agent.location.to_record().to_string().c_str(),
      self_->agent.orientation.to_record().to_string().c_str());
  }

  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Greet::analyze:" \
      " Platform initialized. Calculating if move is needed.\n");

    groups::AgentVector targets;
    target_group_->sync();
    target_group_->get_members(targets);

    if (targets.size() != 0)
    {
      pose::Position location(platform_->get_frame());
      location.from_container(self_->agent.location);

      pose::Position target_location(platform_->get_frame());

      pose::Position xy_location (location);
      pose::Position xy_guard_location (guard_location_);
      xy_location.z(0);
      xy_guard_location.z(0);

      std::vector<auctions::AuctionMinimumDistance> distance_auctions(
        targets.size());
      std::vector<auctions::AuctionBids> distance_bids(
        targets.size());

      if (follow_)
      {
        Timer overall_timer;
        overall_timer.start();

        // determine distances to targets
        std::string auction_prefix = "auction.";
        for (size_t i = 0; i < targets.size(); ++i)
        {
          containers::NativeDoubleVector target_container(
            targets[i] + ".location", *knowledge_);
          target_location.from_container(target_container);
          target_location.z(0);

          distance_auctions[i].add_group(guard_group_.get());
          distance_auctions[i].set_knowledge_base(knowledge_);
          distance_auctions[i].set_platform(platform_);

          distance_auctions[i].set_auction_prefix(auction_prefix + targets[i]);
          distance_auctions[i].set_agent_prefix(self_->agent.prefix);
          distance_auctions[i].set_target(target_location);

          // under the hood, we're using containers::Map and variables::Agents
          // for the entire group. There are a couple of things to learn here.
          // calculate_bids(), even though it is semantically accurate, will
          // cause unnecessary computation since the other agents are modifying
          // the value with their own loops. Best solution here is to just bid
          // but we can optimize this way more. We're spending a lot of time in
          // the KB in the current implementation
          //   action          50       100       150       200       250
          //   bid         0.0012s   0.0022s   0.0032s   0.0049s   0.0063s
          //   calc_bids   0.0022s   0.0054s   0.0072s   0.0099s   0.0143s
          distance_auctions[i].bid(KnowledgeRecord(
             xy_location.distance_to(target_location)));
          // distance_auctions[i].calculate_bids();
          distance_auctions[i].get_bids(distance_bids[i]);
          auctions::sort_ascending(distance_bids[i]);
        }

        overall_timer.stop();

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MINOR,
          "gams::algorithms::Greet::analyze:" \
          " Time taken for all target calculate_bids and sorting = %f s.\n",
          overall_timer.duration_ds());
      } 

      if (is_following_)
      {
        target_location.from_container(target_.location);
        pose::Position xy_target_location (target_location);
        xy_target_location.z(0);

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::analyze:" \
          " Currently following target (%s->%s).\n",
          target_.location.get_name().c_str(),
          target_.location.to_record().to_string().c_str());

        pose::Epsilon epsilon(follow_max_distance_);
        is_following_ = epsilon.check_position(
          xy_location, xy_target_location);

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::analyze:"
          " %s target distance check (%f<%f) results in following=%s.\n",
          self_->agent.prefix.c_str(),
          xy_location.distance_to(xy_target_location),
          follow_max_distance_,
          is_following_ ? "true" : "false");

        if (is_following_ && guard_max_follow_distance_ > 0)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MINOR,
            "gams::algorithms::Greet::analyze:"
            " %s max follow distance from guard check (%f<%f).\n",
            self_->agent.prefix.c_str(),
            xy_location.distance_to(xy_guard_location),
            guard_max_follow_distance_);

          pose::Epsilon rubberband(guard_max_follow_distance_);
          is_following_ = rubberband.check_position(
            xy_location, xy_guard_location);

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MINOR,
            "gams::algorithms::Greet::analyze:"
            " %s follow distance from guard check results in following=%s.\n",
            self_->agent.prefix.c_str(),
            is_following_ ? "true" : "false");

        }

        if (!is_following_)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::Greet::analyze:"
            " agent %s is removing name from group %s.\n",
            self_->agent.prefix.c_str(),
            follow_group_->get_prefix().c_str()
          );

          // follow_group_->remove_members({self_->agent.prefix});
          knowledge_->set(
            follow_group_->get_prefix() + ".members." + self_->agent.prefix,
            (Integer)0, knowledge::EvalSettings::DELAY);
        }

        if (is_guarding_)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_ERROR,
            "gams::algorithms::Greet::analyze:"
            " LOGIC ERROR: agent %s is guarding and following.\n",
            self_->agent.prefix.c_str(),
            follow_group_->get_prefix().c_str()
          );
        }
      } // end if is following
      else if (is_guarding_)
      {
        target_location.from_container(target_.location);
        pose::Position xy_target_location (target_location);
        xy_target_location.z(0);

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::analyze:" \
          " Currently guarding target (%s->%s).\n",
          target_.location.get_name().c_str(),
          target_.location.to_record().to_string().c_str());

        // not following and is guarding. Check to see if we need to guard
        groups::AgentVector targets;
        target_group_->sync();
        target_group_->get_members(targets);

        is_guarding_ = guard_epsilon_.check_position(
          xy_location, xy_target_location);

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::analyze:"
          " target distance check results in guarding=%s.\n",
          is_guarding_ ? "true" : "false");

        if (!is_guarding_)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::Greet::analyze:"
            " iterating through targets to see if they need guards.\n");

          // iterate through all targets to see if anything is near
          for (auto target : targets)
          {
            containers::NativeDoubleVector target_container(
              target + ".location", *knowledge_);
            target_location.from_container(target_container);
            target_location.z(0);

            if (guard_epsilon_.check_position(xy_location, target_location))
            {
              target_.init_vars(*knowledge_, target);
              is_guarding_ = true;

              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_MAJOR,
                "gams::algorithms::Greet::analyze:"
                " Found new target to guard at (%s->%s).\n",
                target_.location.get_name().c_str(),
                target_.location.to_record().to_string().c_str());

              break;
            } // end if check position from location to target_location
          } // end for all targets
        } // end if not is guarding

        if (is_guarding_ && follow_)
        {
          // check to see if there are enough followers

          std::string new_group;

          std::stringstream buffer;
          buffer << follow_group_prefix_;
          buffer << target_.prefix;
          buffer << follow_group_suffix_;
          new_group = buffer.str();

          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MINOR,
            "gams::algorithms::Greet::analyze:"
            " GUARDING. Checking to see if we should follow in group %s.\n",
            new_group.c_str());

          groups::AgentVector followers;

          if (follow_group_.get() == 0 ||
              follow_group_->get_prefix() != new_group)
          {
            madara_logger_ptr_log(gams::loggers::global_logger.get(),
              gams::loggers::LOG_MINOR,
              "gams::algorithms::Greet::analyze:"
              " last follow group is different from new group (%s)."
              " Creating new group.\n",
              new_group.c_str());

            // don't use factory as we need to force a transient group
            follow_group_.reset(
              new groups::GroupTransient(new_group, knowledge_));
          }
          else
          {
            follow_group_->sync();
          }

          follow_group_->get_members(followers);

          if (groups::find_member_index(self_->agent.prefix, followers) < 0)
          {
            if (followers.size() < (size_t)follow_max_agents_)
            {
              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_MINOR,
                "gams::algorithms::Greet::analyze:"
                " Adding self prefix (%s) to group (%s). Size is %zu.\n",
                self_->agent.prefix.c_str(),
                new_group.c_str(),
                followers.size());

              followers.clear();
              followers.push_back(self_->agent.prefix);
              follow_group_->add_members (followers);

              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_MAJOR,
                "gams::algorithms::Greet::analyze:"
                " Finished adding self %s to group %s. New size is %zu.\n",
                self_->agent.prefix.c_str(),
                new_group.c_str(),
                followers.size());

              is_following_ = true;
              is_guarding_ = false;
            }
            else
            {
              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_MINOR,
                "gams::algorithms::Greet::analyze:"
                " Already have enough followers (%zu) for group %s.\n",
                followers.size(), follow_group_->get_prefix().c_str()
              );
            }
          }
          else
          {
            madara_logger_ptr_log(gams::loggers::global_logger.get(),
              gams::loggers::LOG_ERROR,
              "gams::algorithms::Greet::analyze:"
              " LOGIC ERROR: agent %s is already in follow group %s.\n",
              self_->agent.prefix.c_str(),
              follow_group_->get_prefix().c_str()
            );
          }
        } // end if guarding and follow is enabled
      } // end if is guarding
      else
      {
        // iterate through all targets to see if anything is near
        for (auto target : targets)
        {
          containers::NativeDoubleVector target_container(
            target + ".location", *knowledge_);
          target_location.from_container(target_container);
          target_location.z(0);

          if (guard_epsilon_.check_position(
              xy_guard_location, target_location))
          {
            target_.init_vars(*knowledge_, target);
            is_guarding_ = true;

            madara_logger_ptr_log(gams::loggers::global_logger.get(),
              gams::loggers::LOG_MAJOR,
              "gams::algorithms::Greet::analyze:"
              " Guarding at [%s] due to target %s at [%s].\n",
              guard_location_.to_string().c_str(),
              target_.prefix.c_str(),
              target_location.to_string().c_str());

            break;
          } // end if location is within episolon of target_location
        } // end for all targets

        if (!is_guarding_)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::Greet::analyze:"
            " Not guarding or following. Should return home.\n",
            guard_location_.to_string().c_str(),
            target_.prefix.c_str(),
            target_location.to_string().c_str());
        }
      } // end not following or guarding yet
    } // end if targets group has members
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Greet::analyze:"
        " Targets group (%s) has no members. Nothing to do.\n",
        target_group_->get_prefix().c_str());

      is_following_ = false;
      is_guarding_ = false;
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Greet::analyze:"
      " Final analysis: Guarding=%s, Following=%s.\n",
      is_guarding_ ? "true" : "false",
      is_following_ ? "true" : "false");
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Greet::analyze:" \
      " Platform not initialized. Unable to analyze.\n");
  }
  return OK;
}
      
/**
 * Move to next location if next_position_ is valid
 */
int
gams::algorithms::Greet::execute(void)
{
  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Greet::execute:" \
      " Platform initialized. Guarding=%s, Following=%s.\n",
      is_guarding_ ? "true" : "false",
      is_following_ ? "true" : "false");

    if (is_following_)
    {
      pose::Position target (platform_->get_frame());
      target.from_container (target_.location);

      pose::ReferenceFrame cur_frame(target_.prefix, target);

      groups::AgentVector followers;
      follow_group_->get_members(followers);
      
      int found = groups::find_member_index(self_->agent.prefix, followers);

      gams::pose::Position next_pos (cur_frame,
        0, 0, target.z() + 10.0 + found);
      platform_->move(next_pos, platform_->get_accuracy());

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Greet::execute:" \
        " FOLLOW: agent %s moving above target %s [%s] at offset [%s].\n",
        self_->agent.prefix.c_str(),
        target_.prefix.c_str(),
        target.to_string().c_str(),
        next_pos.to_string().c_str()
      );
    }
    else if (is_guarding_)
    {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::execute:" \
          " GUARD: agent %s moving to guard location [%s].\n",
          self_->agent.prefix.c_str(),
          guard_location_.to_string().c_str()
        );

      platform_->move(guard_location_, platform_->get_accuracy());
    }
    else
    {
      if (use_agent_home_)
      {
        // return home
        pose::Position home (platform_->get_frame());
        home.from_container(self_->agent.home);

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::execute:" \
          " HOME: agent %s moving to %s.home [%s].\n",
          self_->agent.prefix.c_str(),
          self_->agent.prefix.c_str(),
          home.to_string().c_str()
        );

        platform_->move(home, platform_->get_accuracy());
      }
      else
      {
        // return to custom home/perch

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Greet::execute:" \
          " HOME: agent %s moving to custom home [%s].\n",
          self_->agent.prefix.c_str(),
          home_location_.to_string().c_str()
        );

        platform_->move(home_location_, platform_->get_accuracy());
      }
      
    }
    

    ++executions_;
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::Greet::execute:" \
      " ERROR: Platform not initialized. Unable to execute.\n");
  }
  return 0;
}

/**
 * Nothing to do in planning stage
 */
int
gams::algorithms::Greet::plan(void)
{
  return 0;
}
