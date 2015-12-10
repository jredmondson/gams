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
 * @file ZoneCoverage.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Algorithm for zone coverages with groups
 **/

#ifndef _GAMS_ALGORITHMS_ZONE_COVERAGE_H_
#define _GAMS_ALGORITHMS_ZONE_COVERAGE_H_

#include <vector>
#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/utility/Location.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Barrier.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for covering an area in formation with a synchronous
    * model of computation. Allows specification of arbitrary group or swarm.
    *
    * Supports the following formation types, with the following names:
    *
    * "line": protectors arrange themselves in a line perpendicular to line
    *   connecting enemy to asset, and parallel to the ground; supports only
    *   one enemy and one asset (only first in each group will be used)
    **/
    class GAMSExport ZoneCoverage : public BaseAlgorithm
    {
    public:

      /**
       * Constructor
       * @param  protectors   name of group of protecting agents
       * @param  assets       name of group of agents to protect
       * @param  enemies      name of group of attacking agents
       * @param  formation    name of formation
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       **/
      ZoneCoverage (
        const std::string &protectors,
        const std::string &assets,
        const std::string &enemies,
        const std::string &formation,
        double buffer,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~ZoneCoverage ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const ZoneCoverage & rhs);
      
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
      
    protected:
      std::string protector_group_;
      std::string asset_group_;
      std::string enemy_group_;

      madara::knowledge::containers::StringVector protectors_;
      madara::knowledge::containers::StringVector assets_;
      madara::knowledge::containers::StringVector enemies_;
      std::string formation_;
      double buffer_;

      int index_;

      typedef utility::Location (ZoneCoverage::*formation_func)() const;

      formation_func form_func_;

      utility::Location line_formation() const;
      utility::Location arc_formation() const;
      utility::Location onion_formation() const;

      static formation_func get_form_func(const std::string &form_name);

      typedef std::vector<madara::knowledge::containers::NativeDoubleArray> MadaraArrayVec;

      MadaraArrayVec asset_loc_cont_;
      MadaraArrayVec enemy_loc_cont_;

      std::vector<utility::Location> asset_locs_;
      std::vector<utility::Location> enemy_locs_;
      utility::Location next_loc_;

    private:
      madara::knowledge::containers::StringVector get_group(const std::string &name) const;
      void update_arrays(const madara::knowledge::containers::StringVector &names,
                         MadaraArrayVec &arrays) const;
      void update_locs(const MadaraArrayVec &arrays,
                       std::vector<utility::Location> &locs) const;
      int get_index() const;
    };
    
    /**
     * A factory class for creating Formation Coverage algorithms
     **/
    class GAMSExport ZoneCoverageFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Formation Coverage Algorithm.
       * @param   args      args come in pairs. The first arg is the
       *                    name of an arg. The second arg is the value
       *                    of the arg.<br>
       *                    No args defined for this factory yet.
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
      virtual BaseAlgorithm * create (
        const madara::knowledge::KnowledgeVector & args,
        madara::knowledge::KnowledgeBase * knowledge,
        platforms::BasePlatform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Agents * agents);
    };
  }
}

#endif // _GAMS_ALGORITHMS_FORMATION_SYNC_H_
