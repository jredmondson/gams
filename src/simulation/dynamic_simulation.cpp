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
 *      are those of the author (s) and do not necessarily reflect the views of
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
 * @file dynamic_simulation.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Sets up a vrep simulation environment
 **/

#include "madara/knowledge_engine/Knowledge_Base.h"
#include "madara/utility/Utility.h"

#include "gams/utility/Region.h"
using gams::utility::Region;
#include "gams/utility/Prioritized_Region.h"
using gams::utility::Prioritized_Region;
#include "gams/utility/Search_Area.h"
using gams::utility::Search_Area;
#include "gams/utility/Position.h"
using gams::utility::Position;
#include "gams/utility/GPS_Position.h"
using gams::utility::GPS_Position;
#include "gams/variables/Sensor.h"
using gams::variables::Sensor;

extern "C" {
#include "extApi.h"
}

#ifndef  _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <cmath>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <set>
using std::set;
#include <ctime>
using std::time_t;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

// default transport settings
std::string host ("");
const std::string default_multicast ("239.255.0.1:4150");
Madara::Transport::QoS_Transport_Settings settings;

// create shortcuts to MADARA classes and namespaces
namespace engine = Madara::Knowledge_Engine;
typedef Madara::Knowledge_Record Record;
typedef Record::Integer Integer;

// controller variables
double period = 1.0;
double sim_time = 50.0;

// vrep connection information
string vrep_host = "127.0.0.1";
int vrep_port = 19905;

// SW corner of simulation
double sw_lat = 40.442824;
double sw_long = -79.940967;

// NE corner of simulation
double ne_lat = 40.44355;
double ne_long = -79.939626;

// search area identifier
string search_area_id = "search_area.1";

// Agent information
unsigned int num_agents = 0;

// place plants or not
bool plants = false;
vector<string> regions;

// use gps coords
bool gps = false;

// madara init
string madara_commands = "";

// number of coverages
unsigned int num_coverages = 1;

// ground surface to use
unsigned int surface = 0;

/**
 * Print out program usage
 **/
void print_usage (string prog_name)
{
  cerr << "Program summary for " << prog_name << ":" << endl;
  cerr << endl;
  cerr << "   Creates VREP simulation environment of specific size and" << endl;
  cerr << "   launches simulated gams agents" << endl;
  cerr << endl;
  cerr << "   [-c | --coverages <number>]" << endl;
  cerr << "       number of coverages to collect data for" << endl;
  cerr << "   [-g | --gps]" << endl;
  cerr << "       use gps coords (instead of vrep)" << endl;
  cerr << "   [-l | --log-level <number>]" << endl;
  cerr << "       MADARA log level, 1-10" << endl;
  cerr << "   [-m | --madara-file <file(s)>]" << endl;
  cerr << "       madara variable initialization files" << endl;
  cerr << "   [-n | --num-agents <number>]" << endl;
  cerr << "       number of agents that will be launched" << endl;
  cerr << "   [--north-east <coords>]" << endl;
  cerr << "       northeast corner coordinates, ex. \"40,-72\"" << endl;
  cerr << "   [-p | --plants]" << endl;
  cerr << "       place plants as position markers" << endl;
  cerr << "   [--south-west <coords>]" << endl;
  cerr << "       southwest corner coordinates, ex. \"40,-72\"" << endl;
  cerr << "   [-s | --surface <concrete, water> ] " << endl;
  cerr << "       ground surface to be used" << endl;
  cerr << "   [-v | --vrep <ip_address> <port>]" << endl;
  cerr << "       vrep connection information" << endl;

  exit (0);
}

// handle command line arguments
void handle_arguments (int argc, char** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == "-c" || arg1 == "--coverages")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        sscanf (argv[i + 1], "%u", &num_coverages);
      else
        print_usage (argv[0]);
      ++i;
    }
    else if (arg1 == "-g" || arg1 == "--gps")
    {
      gps = true;
    }
    else if (arg1 == "-l" || arg1 == "--log-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        sscanf (argv[i + 1], "%d", &MADARA_debug_level);
      else
        print_usage (argv[0]);
      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--madara-file")
    {
      madara_commands = "";
      bool files = false;
      for (;i + 1 < argc && argv[i + 1][0] != '-'; ++i)
      {
        madara_commands += Madara::Utility::file_to_string (argv[i + 1]);
        madara_commands += ";\r\n";
        files = true;
      }

      if (!files)
        print_usage (argv[0]);
    }
    else if (arg1 == "-n" || arg1 == "--num-agents")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        sscanf (argv[i + 1], "%u", &num_agents);
      else
        print_usage (argv[0]);
      ++i;
    }
    else if (arg1 == "--north-east")
    {
      if (i + 1 < argc)
        sscanf (argv[i + 1], "%lf,%lf", &ne_lat, &ne_long);
      else
        print_usage (argv[0]);
      ++i;
    }
    else if (arg1 == "-p" || arg1 == "--plants")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        plants = true;
        char temp[50];
        char arg[500];
        strcpy (arg, argv[i + 1]);
        while (sscanf(arg, "%[^,],%s", temp, arg) > 1)
          regions.push_back (temp);
        regions.push_back (temp);
      }
      ++i;
    }
    else if (arg1 == "--south-west")
    {
      if (i + 1 < argc)
        sscanf (argv[i + 1], "%lf,%lf", &sw_lat, &sw_long);
      else
        print_usage (argv[0]);
      ++i;
    }
    else if (arg1 == "-s" || arg1 == "--surface")
    {
      if (i + 1 < argc)
      {
        std::string arg ( argv[i + 1] );
	if (arg == "concrete")
          surface = 0;
        else if( arg == "water" )
          surface = 1; 
        else
         print_usage (argv[0]);
      }
     else
      print_usage (argv[0]);
     ++i;


    }
    else if (arg1 == "-v" || arg1 == "--vrep")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        vrep_host = argv[i + 1];
      else
        print_usage (argv[0]);
      ++i;

      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
          sscanf (argv[i + 1], "%d", &vrep_port);
          ++i;
      }
    }
    else
    {
      print_usage (argv[0]);
    }
  }
}

/**
 * Get dimensions of environment using SW corner and NE corner
 * VREP uses y coord for N/S and x for E/W
 **/
void get_dimensions (double &max_x, double &max_y,
  Madara::Knowledge_Engine::Knowledge_Base& knowledge)
{
  // check knowledge base
  string sw_position = knowledge.get (".vrep_sw_position").to_string ();
  string ne_position = knowledge.get (".vrep_ne_position").to_string ();
  if (sw_position != "0")
  {
    sscanf (sw_position.c_str (), "%lf,%lf", &sw_lat, &sw_long);
    sscanf (ne_position.c_str (), "%lf,%lf", &ne_lat, &ne_long);
  }
  
  // assume the Earth is a perfect sphere
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // get y/lat first
  max_y = (ne_lat - sw_lat) / 360.0 * EARTH_CIRCUMFERENCE;
  
  // assume the meters/degree longitude is constant throughout environment
  // convert the longitude/x coordinates
  double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (sw_lat));
  double circumference = 2 * r_prime * M_PI;
  max_x = (ne_long - sw_long) / 360.0 * circumference;
}

/**
 * Put plants around a region
 * @param region region object to put plants around
 */
void put_plants (Madara::Knowledge_Engine::Knowledge_Base& knowledge, 
   const Region& reg, const int& client_id)
{
  string model_file = getenv ("VREP_ROOT");
  model_file += "/models/furniture/plants/indoorPlant.ttm";

  for (size_t j = 0; j < reg.vertices.size (); ++j)
  {
    size_t next = (j + 1) % reg.vertices.size ();
    string sw_position = knowledge.get (".vrep_sw_position").to_string ();
    gams::utility::GPS_Position origin;
    double latitude, longitude;
    sscanf (sw_position.c_str (), "%lf,%lf", &latitude, &longitude);
    origin.latitude (latitude); origin.longitude (longitude);

    const gams::utility::GPS_Position gps_pos_1 = reg.vertices[j];
    const gams::utility::GPS_Position gps_pos_2 = reg.vertices[next];
    const gams::utility::Position pos_1 = gps_pos_1.to_position (origin);
    const gams::utility::Position pos_2 = gps_pos_2.to_position (origin);
    const double delta_x = pos_2.x - pos_1.x;

    const unsigned int NUM_PLANTS_PER_SIDE = 5;
    for (unsigned int k = 0; k < NUM_PLANTS_PER_SIDE; ++k)
    {
      double plant_x, plant_y;
      if (delta_x != 0)
      {
        const double m = (pos_2.y - pos_1.y) / delta_x;
        const double k_del_x = k * delta_x / NUM_PLANTS_PER_SIDE;
        plant_x = pos_1.x + k_del_x;
        plant_y = pos_1.y + m * k_del_x;
      }
      else // vertical line
      {
        plant_x = pos_1.x;
        plant_y = pos_1.y + (pos_2.y - pos_1.y) * k / NUM_PLANTS_PER_SIDE;
      }

      // find where it should go
      simxFloat pos[3];
      pos[0] = plant_y;
      pos[1] = plant_x;
      pos[2] = 0.4;
  
      // load object
      int node_id;
      if (simxLoadModel (client_id, model_file.c_str (), 0, &node_id,
        simx_opmode_oneshot_wait) != simx_error_noerror)
      {
        cerr << "failure loading plant model" << endl;
        exit (0);
      }
  
      // move object
      simxSetObjectPosition (client_id, node_id, sim_handle_parent, pos,
        simx_opmode_oneshot_wait);
    }
  }
}

/**
 * Create environment in vrep
 * Add floors and plants as visible markers
 * @param client_id id for vrep connection
 */
void create_environment (const int& client_id,
  Madara::Knowledge_Engine::Knowledge_Base& knowledge)
{
  // close scene
  simxCloseScene (client_id, simx_opmode_oneshot_wait);
  string scene (getenv ("GAMS_ROOT"));
  scene += "/resources/vrep/starting.ttt";
  simxLoadScene (client_id, scene.c_str(), 0, simx_opmode_oneshot_wait);

  // inform clients they can interact with scene now
  knowledge.set ("vrep_ready", Integer (1));

 //Add concrete
 if( surface == 0) 
 {
   // find environment parameters
   const int floor_size = 5;	
   double max_x, max_y;
   get_dimensions (max_x, max_y, knowledge);
   cout << "creating environment of size " << max_x << " x " << max_y << "...";
   int num_x = max_x / floor_size + 2;
   int num_y = max_y / floor_size + 2;
  
   // load floor models
   string model_file (getenv ("VREP_ROOT"));
   model_file += "/models/infrastructure/floors/5mX5m concrete floor.ttm";
   for (int i = 0; i < (num_x * num_y); ++i)
   {
     // find where it should go
     simxFloat pos[3];
 	pos[0] = (i / num_y) * floor_size;
	pos[1] = (i % num_y) * floor_size;
     pos[2] = 0;

     // load object
     int node_id;
     if (simxLoadModel (client_id, model_file.c_str (), 0, &node_id,
       simx_opmode_oneshot_wait) != simx_error_noerror)
     {
       cerr << "failure loading floor model" << endl;
       exit (0);
     }

     // move object
     simxSetObjectPosition (client_id, node_id, sim_handle_parent, pos,
       simx_opmode_oneshot_wait);
   }
  }
  //Add water
  else
  {
   
    //Water tile size is 10x10 
    const int water_size = 10;	
    double max_x, max_y;
    get_dimensions (max_x, max_y, knowledge);
    cout << "creating environment of size " << max_x << " x " << max_y << "...";
    int num_x = max_x / water_size + 2;
    int num_y = max_y / water_size + 2;


    string water_file (getenv ("GAMS_ROOT"));
    water_file += "/resources/vrep/water_functional.ttm";
    // find where it should go
    simxFloat pos[3];
	pos[0] = max_x/2;
	pos[1] = max_y/2;
    pos[2] = 0;

    // load object
    int node_id;
    if (simxLoadModel (client_id, water_file.c_str (), 0, &node_id,
      simx_opmode_oneshot_wait) != simx_error_noerror)
    {
      cerr << "failure loading floor model" << endl;
      exit (0);
    }
    //Set functional element visibility off
    simxSetModelProperty(client_id, node_id, sim_modelproperty_not_visible,
      simx_opmode_oneshot_wait);
     	
    // move object
    simxSetObjectPosition (client_id, node_id, sim_handle_parent, pos,
      simx_opmode_oneshot_wait);
    // load water scene objects
    string model_file (getenv ("VREP_ROOT"));
    model_file += "/models/nature/water surface.ttm";
    for (int i = 0; i < (num_x * num_y); ++i)
    {
      // find where it should go
      simxFloat pos[3];
 	pos[0] = (i / num_y) * water_size;
 	pos[1] = (i % num_y) * water_size;
      pos[2] = 0;

      // load object
     int node_id;
     if (simxLoadModel (client_id, model_file.c_str (), 0, &node_id,
       simx_opmode_oneshot_wait) != simx_error_noerror)
     {
       cerr << "failure loading water model" << endl;
       exit (0);
     }

     // move object
     simxSetObjectPosition (client_id, node_id, sim_handle_parent, pos,
       simx_opmode_oneshot_wait);
   }
  }
  cout << "done" << endl;
  
  // load plants as position markers
  if (plants)
  {
    cout << "placing plants as markers...";

    // paint each vertex
    for (size_t i = 0; i < regions.size (); ++i)
    {
      if (regions[i].find ("region") != std::string::npos)
      {
        gams::utility::Region reg =
          gams::utility::parse_region (knowledge, regions[i]);
        put_plants (knowledge, reg, client_id);
      }
      else // search_area
      {
        gams::utility::Search_Area search =
          gams::utility::parse_search_area (knowledge, regions[i]);
        vector<Prioritized_Region> search_regions = search.get_regions ();
        for (size_t j = 0; j < search_regions.size (); ++j)
          put_plants (knowledge, search_regions[j], client_id);
      }
    }

    cout << "done" << endl;
  }
}

/**
 * Wait for all simulated control loops to sync, then start vrep
 **/
void start_simulator (const int & client_id,
  Madara::Knowledge_Engine::Knowledge_Base & knowledge)
{
  // wait for all processes to get up
  std::stringstream buffer;
  buffer << "(vrep_ready = 1) && S0.init";
  for (unsigned int i = 1; i < num_agents; ++i)
    buffer << " && S" << i << ".init";
  string expression = buffer.str ();
  Madara::Knowledge_Engine::Compiled_Expression compiled;
  compiled = knowledge.compile (expression);
  cout << "waiting for " << num_agents << " agent(s) to come online...";
  knowledge.wait (compiled);
  cout << "done" << endl;

  // start the simulation
  cout << "starting simulation...";
  simxSetBooleanParameter (client_id, sim_boolparam_hierarchy_visible, false, simx_opmode_oneshot);
  simxSetBooleanParameter (client_id, sim_boolparam_browser_visible, false, simx_opmode_oneshot);
  simxStartSimulation (client_id, simx_opmode_oneshot_wait);
  cout << "done" << endl;

  // inform simulated control loops to begin
  cout << "informing agents to continue...";
  knowledge.set ("begin_sim", Integer (1));
  cout << "done" << endl;
}

void
time_to_full_coverage (Madara::Knowledge_Engine::Knowledge_Base& knowledge, 
  const Search_Area& search)
{
  // get sensors and discrete area
  GPS_Position origin;
  Madara::Knowledge_Engine::Containers::Native_Double_Array origin_container;
  origin_container.set_name ("sensor.coverage.origin", knowledge, 3);
  origin.from_container (origin_container);
  Sensor coverage_sensor ("coverage", &knowledge, 2.5, origin);
  const set<Position> valid_positions = coverage_sensor.discretize (
    search);

  // record start time
  time_t start = time (NULL);

  // check for multiple rounds of coverage
  for (unsigned int i = 0; i < num_coverages; ++i)
  {
    time_t inner_start = time (NULL);

    unsigned int num_not_covered = 1;
    while (num_not_covered > 0)
    {
      Madara::Utility::sleep (1);

      num_not_covered = 0;
      for (set<Position>::const_iterator it = valid_positions.begin ();
        it != valid_positions.end (); ++it)
      {
        if (coverage_sensor.get_value (*it) == 0)
          ++num_not_covered;
      }

      double coverage = double(valid_positions.size() - num_not_covered) / 
        valid_positions.size ();
      cout << "coverage: " << (coverage * 100) << "%" << endl;
    }
  
    // find complete time
    time_t inner_end = time (NULL);
    cout << inner_end - inner_start << " seconds for full coverage " << (i + 1) << endl;

    // reset coverage
    for (set<Position>::const_iterator it = valid_positions.begin ();
      it != valid_positions.end (); ++it)
    {
      coverage_sensor.set_value (*it, 0);
    }
  }

  time_t end = time (NULL);
  cout << end - start << " seconds for " << num_coverages << " full coverages" << endl;
}

/**
 * entry point
 **/
int main (int argc, char ** argv)
{
  // handle all user arguments
  handle_arguments (argc, argv);

  // create knowledge base
  settings.type = Madara::Transport::MULTICAST;
  if (settings.hosts.size () == 0)
  {
    // setup default transport as multicast
    settings.hosts.push_back (default_multicast);
  }
  Madara::Knowledge_Engine::Knowledge_Base knowledge (host, settings);
  if (madara_commands != "")
    knowledge.evaluate (madara_commands,
      Madara::Knowledge_Engine::Eval_Settings (false, true));

  // connect to vrep
  cout << "connecting to vrep...";
  int client_id = 
    simxStart (vrep_host.c_str (), vrep_port, true, true, 2000, 5);
  if (client_id == -1)
  {
    cerr << "failure connecting to vrep" << endl;
    exit (-1);
  }
  simxStopSimulation (client_id, simx_opmode_oneshot);
  cout << "done" << endl;

  // create environment and start simulation
  create_environment (client_id, knowledge);
  if (num_agents > 0)
    start_simulator (client_id, knowledge);

  // close connection to vrep
  cout << "closing vrep connection...";
  simxFinish (client_id);
  cout << "done" << endl;

  // data collection
  Search_Area search = gams::utility::parse_search_area (knowledge, search_area_id);
  time_to_full_coverage (knowledge, search);

  // exit
  return 0;
}
