/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Sensor.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the sensor-prefixed MADARA variables
 **/

#ifndef   _GAMS_VARIABLES_SENSOR_H_
#define   _GAMS_VARIABLES_SENSOR_H_

#include <vector>
#include <map>
#include <string>

#include "gams/GamsExport.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/containers/Map.h"
#include "madara/knowledge/KnowledgeBase.h"

#include "gams/pose/SearchArea.h"
#include "gams/pose/GPSFrame.h"
#include "gams/pose/CartesianFrame.h"

#include <set>
using std::set;

namespace gams
{
  namespace variables
  {
    /**
    * A container for sensor information
    **/
    class GAMS_EXPORT Sensor
    {
    public:
      /**
       * Constructor
       **/
      Sensor();

      /**
       * Constructor
       **/
      Sensor(const std::string & name,
        madara::knowledge::KnowledgeBase * knowledge,
        const double & range = 0.0,
        const pose::Position & origin = pose::Position(pose::gps_frame(), DBL_MAX, DBL_MAX));

      /**
       * Destructor
       **/
      ~Sensor();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator=(const Sensor & rhs);

      /**
       * Discretize a search area into index positions inside search area
       * @param region  region to discretize
       * @return set of index positions considered inside search area
       **/
      set<pose::Position> discretize(
        const pose::Region & region);

      /**
       * Discretize a search area into index positions inside search area
       * @param area  area to discretize
       * @return set of index positions considered inside search area
       **/
      set<pose::Position> discretize(
        const pose::SearchArea & area);
      
      /**
       * Get the length of the side of each discretized cell
       * @return discretization value
       */
      double get_discretization() const;

      /**
       * Gets GPS position from index position
       * @param index   index location in cartesian location on sensor map
       * @return GPSPosition of index position
       **/
      pose::Position get_gps_from_index(
        const pose::Position & index);

      /**
       * Gets current location on sensor map
       * @param pos current GPS location
       * @return current location in cartesian location on sensor map
       **/
      pose::Position get_index_from_gps(
        const pose::Position & pos);

      /**
       * Gets name
       * @return name of sensor
       **/
      std::string get_name() const;

      /**
       * Gets origin
       * @return GPS origin
       **/
      pose::Position get_origin();

      /**
       * Gets range in meters
       * @return sensor range
       **/
      double get_range() const;

      /**
       * Gets value at location
       * @param pos   position to get
       * @return sensor value at pos
       **/
      double get_value(const pose::Position& pos);

      /**
       * Sets origin
       * @param origin  new origin
       **/
      void set_origin(const pose::Position& origin);

      /**
       * Sets range in meters
       * @param range new range
       **/
      void set_range(const double& range);

      /**
       * Sets value at a point with position
       * @param pos     position to set
       * @param val     value to set at position
       * @param settings  settings to use for mutating value
       **/
      void set_value(const pose::Position& pos, const double& val,
        const madara::knowledge::KnowledgeUpdateSettings& settings =
          madara::knowledge::KnowledgeUpdateSettings());
      /**
       * Initializes the variables
       * @param name      name of the sensor
       * @param knowledge the knowledge base
       * @param range     the range
       * @param origin    the GPS origin
       **/
      void init_vars(const std::string & name,
        madara::knowledge::KnowledgeBase* knowledge,
        const double & range = 0.0,
        const pose::Position & origin = pose::Position(pose::gps_frame(), DBL_MAX, DBL_MAX));

    protected:
      /**
       * Convert index position to string index
       * @param pos   gps position
       * @return string index into map
       **/
      std::string index_pos_to_index(const pose::Position& pos) const;

      void regenerate_local_frame(void);

      /**
       * Initialize madara containers
       */
      void init_vars();

      /// the map of locations to sensor value
      madara::knowledge::containers::Map value_;

      /// knowledge base
      madara::knowledge::KnowledgeBase* knowledge_;

      /// the range of the sensor, determines discretization
      madara::knowledge::containers::Double range_;

      /// name of the sensor
      std::string name_;

      /// origin for index calculations
      madara::knowledge::containers::NativeDoubleArray origin_;

      /// local cartesian frame
      pose::ReferenceFrame local_frame_;
    };

    /// a map of sensor names to the sensor information
    typedef  std::map <std::string, Sensor*>   Sensors;

    /// a list of sensor names
    typedef  std::vector <std::string>        SensorNames;
  }
}

#endif // _GAMS_VARIABLES_SENSOR_H_
