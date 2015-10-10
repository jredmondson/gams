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

#include "SnakeAreaCoverage.h"

#include <cmath>
#include <string>
#include <vector>

#include "gams/utility/GPSPosition.h"
#include "gams/utility/Region.h"
#include "gams/utility/Position.h"

using std::vector;
using std::string;

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::SnakeAreaCoverageFactory::create (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && self && devices)
  {
    if (args.size () >= 1)
    {
      if (args[0].is_string_type ())
      {
        if (args.size () == 2)
        {
          if (args[1].is_double_type () || args[1].is_integer_type ())
          {
            result = new area_coverage::SnakeAreaCoverage (
              args[0].to_string () /* search area id*/,
              ACE_Time_Value (args[1].to_double ()) /* exec time */,
              knowledge, platform, sensors, self, devices);
          }
          else
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_ERROR,
               "gams::algorithms::SnakeAreaCoverageFactory::create:" \
              " invalid second arg, expected double\n");
          }
        }
        else
        {
          result = new area_coverage::SnakeAreaCoverage (
            args[0].to_string () /* search area id*/,
            ACE_Time_Value (0.0) /* run forever */,
            knowledge, platform, sensors, self, devices);
        }
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::algorithms::SnakeAreaCoverageFactory::create:" \
          " invalid first arg, expected string\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::SnakeAreaCoverageFactory::create:" \
        " expected 1 or 2 args\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::SnakeAreaCoverageFactory::create:" \
      " invalid knowledge, sensors, self, or devices parameters\n");
  }

  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::SnakeAreaCoverageFactory::create:" \
      " unknown error creating algorithm\n");
  }

  return result;
}

/**
 * SnakeAreaCoverage is a precomputed area coverage algorithm. The agent
 * traverses parallel lines in the region starting with the longest edge.
 */
gams::algorithms::area_coverage::SnakeAreaCoverage::SnakeAreaCoverage (
  const string& region_id,
  const ACE_Time_Value& e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Devices * devices) :
  BaseAreaCoverage (knowledge, platform, sensors, self, devices, e_time),
  cur_waypoint_ (0)
{
  status_.init_vars (*knowledge, "sac", self->id.to_integer ());
  status_.init_variable_values ();

  // setup
  compute_waypoints (region_id);

  generate_new_position ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::SnakeAreaCoverage::constructor:" \
    " selected \"%s\"\n", next_position_.to_string ().c_str ());

  cerr << "exec_time_: " << exec_time_ << endl;
  cerr << "end_time_: " << end_time_ << endl;
}

gams::algorithms::area_coverage::SnakeAreaCoverage::~SnakeAreaCoverage ()
{
}

void
gams::algorithms::area_coverage::SnakeAreaCoverage::operator= (
  const SnakeAreaCoverage& rhs)
{
  if (this != &rhs)
  {
    this->waypoints_ = rhs.waypoints_;
    this->cur_waypoint_ = rhs.cur_waypoint_;
    this->BaseAreaCoverage::operator= (rhs);
  }
}

/**
 * The next destination is simply the next point in the list
 */
void
gams::algorithms::area_coverage::SnakeAreaCoverage::generate_new_position ()
{
  next_position_ = waypoints_[cur_waypoint_];
  cur_waypoint_ = (cur_waypoint_ + 1) % waypoints_.size ();
}

/**
 * The endpoints of each parallel lines are found and stored in order.
 */
void
gams::algorithms::area_coverage::SnakeAreaCoverage::compute_waypoints (
  const string& region_id)
{
  // get region information
  utility::Region region;
  region.from_container (*knowledge_, region_id);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::SnakeAreaCoverage::compute_waypoints:" \
    " using region \"%s\"\n", region.to_string ().c_str ());

  // convert to equirectangular projection coordinates
  const size_t num_edges = region.vertices.size ();
  vector<utility::Position> positions;
  const utility::GPSPosition reference = region.vertices[0];
  for (size_t i = 0; i < region.vertices.size (); ++i)
    positions.push_back (region.vertices[i].to_position (reference));

  // find longest edge
  size_t longest_edge = 0;
  double max_dist = positions[0].distance_to_2d (positions[1]);
  for(size_t i = 1; i < num_edges; ++i)
  {
    double dist = positions[i].distance_to_2d (
      positions[(i + 1) % num_edges]);
    if(dist > max_dist)
    {
      max_dist = dist;
      longest_edge = i;
    }
  }

  // starting points are vertices of longest edge
  utility::GPSPosition temp = utility::GPSPosition::to_gps_position (
    positions[longest_edge], reference);
  temp.altitude (self_->device.desired_altitude.to_double ());
  waypoints_.push_back (temp);
  temp = utility::GPSPosition::to_gps_position (
    positions[(longest_edge + 1) % num_edges], reference);
  temp.altitude (self_->device.desired_altitude.to_double ());
  waypoints_.push_back (temp);

  // determine shift direction
  const double shift = 2.5; // distance in meters between parallel lines
  const utility::Position p_0 = positions[longest_edge];
  const utility::Position p_1 = positions[(longest_edge + 1) % num_edges];

  /**
   * Assuming the bounding area is convex, only one of these loops will 
   * actually do work, the other will fail out the first time through the 
   * while loop due to not finding an intercept.
   *
   * The working loop will find all the snaking waypoints
   **/
  for (int dir = 0; dir < 2; ++dir)
  {
    // loop until no more intercepts are found
    int intercept_idx = -1; // arbitrary non-zero value so we enter loop
    unsigned int loop = 0; // loop counter
    while(intercept_idx != 0)
    {
      intercept_idx = 0;
      ++loop;

      /**
       * Check each edge for intercepts
       * Since we are assuming this is a convex polygon, it can have at most
       * two intercepts
       */
      utility::Position intercepts[2];
      for(size_t i = 1; i < num_edges && intercept_idx < 2; ++i)
      {
        // check current vertex for intercept
        int cur_vertex = (longest_edge + i) % (int)num_edges;

        // beginning and end vertex of intersecing line segment
        const utility::Position p_n_0 = positions[cur_vertex];
        const utility::Position p_n_1 =
          positions[(cur_vertex + 1) % num_edges];
 
        double m_0, m_n;
        utility::Position check;
        bool check_intercept = true;
        // if longest_edge is not vertical
        if (p_0.slope_2d (p_1, m_0))
        {
          // we have a non-vertical line, so find new intercept
          const double delta_b = pow (-1.0, dir) * shift *
            pow (pow (m_0, 2) + 1, 0.5);
 
          // calculate potential waypoint
          if (p_n_0.slope_2d(p_n_1, m_n)) // potential edge not a vertical line
          {
            if(m_n != m_0) // parallel lines can't intersect
            {
              // find intercept of a line parallel to longest_edge and edge
              //    we are checking
              check.y = (m_n * p_0.y - m_n * m_0 * p_0.x +
                m_n * loop * delta_b - m_0 * p_n_0.y + m_0 * m_n * p_n_0.x) /
                (m_n - m_0);
              check.x = (check.y - p_n_0.y + m_n * p_n_0.x) / m_n;
            }
            else // edges are parallel
              check_intercept = false;
          }
          else // edge to check is vertical, perform slopeless waypoint calculation
          {
            check.x = p_n_0.x; // vertical line has same x coord throughout
            check.y = m_0 * check.x + p_0.y - m_0 * p_0.x + loop * delta_b;
          }
        } // end if edge to check is not vertical
        else // longest_edge is vertical line, so just shift the x coord
        {
          /**
           * if potential edge is not a vertical line, then find intercept
           * if potential edge is vertical, then it cannot intercept
           **/
          if (p_n_0.slope_2d(p_n_1, m_n))
          {
            // longest edge is vertical, so just shift the x coord
            check.x = p_0.x + pow(-1.0, (double)dir) * loop * shift;
            check.y = m_n * (check.x - p_n_0.x) + p_n_0.y;
          }
          else
            check_intercept = false;
        } // end else longest_edge is vertical line

        // check if it's actually an intercept
        check.z = self_->device.desired_altitude.to_double (); // TODO: actual altitude control
        if (check_intercept && p_n_0.is_between_2d(p_n_1, check))
          intercepts[intercept_idx++] = check;
      } // end foreach edge

      // found intercepts => go to closest one first
      if (intercept_idx > 1)
      {
        const utility::Position prev = 
          waypoints_[waypoints_.size () - 1].to_position (reference);
        if (prev.distance_to_2d (intercepts[0]) >
            prev.distance_to_2d (intercepts[1]))
        {
          waypoints_.push_back (
            utility::GPSPosition::to_gps_position (intercepts[1], reference));
          waypoints_.push_back (
            utility::GPSPosition::to_gps_position (intercepts[0], reference));
        }
        else
        {
          waypoints_.push_back (
            utility::GPSPosition::to_gps_position (intercepts[0], reference));
          waypoints_.push_back (
            utility::GPSPosition::to_gps_position (intercepts[1], reference));
        }
      }
      else if (intercept_idx > 0)
      {
        waypoints_.push_back (
          utility::GPSPosition::to_gps_position (intercepts[0], reference));
      }
    } // end while still finding intercepts
  } // end for +/- delta_b

  size_t idx = 0;
  for (const utility::GPSPosition& p : waypoints_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::SnakeAreaCoverage::compute_waypoints:" \
      " waypoint %u: \"%s\"\n", idx++, p.to_string ().c_str ());
  }
}
