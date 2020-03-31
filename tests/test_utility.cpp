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
 * @file test_utility.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Tests the functionality of gams::utility classes
 **/

#include <string>
#include <iostream>
#include <assert.h>
#include <vector>
#include <cmath>

#include "gams/utility/OscUdp.h"
#include "gams/utility/Position.h"
#include "gams/utility/GPSPosition.h"
#include "gams/pose/Region.h"
#include "gams/pose/PrioritizedRegion.h"
#include "gams/pose/SearchArea.h"

#include "gams/loggers/GlobalLogger.h"

using gams::utility::GPSPosition;
using gams::utility::Position;
using gams::pose::PrioritizedRegion;
using gams::pose::Region;
using gams::pose::SearchArea;
using std::cout;
using std::endl;
using std::string;
using std::vector;

typedef madara::knowledge::KnowledgeRecord KnowledgeRecord;

void
testing_output (const string& str, const unsigned int& tabs = 0)
{
  for (unsigned int i = 0; i < tabs; ++i)
    cout << "  ";
  cout << "testing " << str << "..." << endl;
}

#ifdef _GAMS_OSC_
void
test_OscUdp ()
{
  testing_output("gams::utility::OscUdp");

  gams::utility::OscUdp osc, osc_server;
  gams::utility::OscUdp::OscMap source_map, dest_map;

  size_t buf_size = 1000;
  size_t read_size = 0;
  char buffer[1000];

  madara::transport::QoSTransportSettings settings;
  settings.hosts.push_back("127.0.0.1:8000");
  settings.hosts.push_back("127.0.0.1:5555");
  settings.type = madara::transport::UDP;

  KnowledgeRecord record;

  record.set_index(1, 0.75);
  record.set_index(0, 1.0);
  source_map["/agent/0/velocity/xy"] = record;

  record.resize(1);
  record.set_index(0, -0.25);
  source_map["/agent/0/velocity/z"] = record;

  source_map["/agent/0/velocity/yaw"] = record;

  record.set_index(2, 1050.00);
  record.set_index(1, 78.1347113);
  record.set_index(0, 42.7234897);
  source_map["/agent/0/pos"] = record;

  record.set_index(2, 50);
  record.set_index(1, 30);
  record.set_index(0, 20);
  source_map["/agent/0/rot"] = record;

  testing_output ("testing pack", 1);
  read_size = osc.pack(buffer, buf_size, source_map);
  testing_output ("testing unpack", 1);
  osc.unpack(buffer, read_size, dest_map);

  std::cout << "    read_size=" << read_size << "\n";
  std::cout << "    source_map:\n";
  for (auto entries : source_map)
  {
    std::cout << "      " << entries.first << " =";
    for (auto arg : entries.second.to_doubles())
    {
      std::cout << " " << arg;
    }
    std::cout << "\n";
  }

  std::cout << "    dest_map:\n";
  for (auto entries : dest_map)
  {
    std::cout << "      " << entries.first << " =";
    for (auto arg : entries.second.to_doubles())
    {
      std::cout << " " << arg;
    }
    std::cout << "\n";
  }

  assert (source_map.size() == dest_map.size() && 
    dest_map["/agent/0/velocity/xy"].size() == 2 &&
    dest_map["/agent/0/velocity/z"].size() == 1 &&
    dest_map["/agent/0/pos"].size() == 3);

  testing_output ("testing client-side create_socket", 1);
  osc.create_socket(settings);

  testing_output ("testing server-side create_socket", 1);

  settings.hosts.clear();
  settings.hosts.push_back("127.0.0.1:5555");
  settings.hosts.push_back("127.0.0.1:8000");

  osc_server.create_socket(settings);

  dest_map.clear();

  testing_output ("testing send", 1);
  osc.send(source_map);

  testing_output ("testing receive", 1);
  osc_server.receive(dest_map);

  testing_output ("testing result", 1);

  std::cout << "    source_map:\n";
  for (auto entries : source_map)
  {
    std::cout << "      " << entries.first << " =";
    for (auto arg : entries.second.to_doubles())
    {
      std::cout << " " << arg;
    }
    std::cout << "\n";
  }

  std::cout << "    dest_map:\n";
  for (auto entries : dest_map)
  {
    std::cout << "      " << entries.first << " =";
    for (auto arg : entries.second.to_doubles())
    {
      std::cout << " " << arg;
    }
    std::cout << "\n";
  }

  assert (source_map.size() == dest_map.size() && 
    dest_map["/agent/0/velocity/xy"].size() == 2 &&
    dest_map["/agent/0/velocity/z"].size() == 1 &&
    dest_map["/agent/0/pos"].size() == 3);

}
#endif // #ifdef _GAMS_OSC_

// TODO: fill out remaining Position function tests
void
test_Position ()
{
  testing_output ("gams::utility::Position");

  // test constructor
  testing_output ("constructor", 1);
  const Position p (1,2,3);
  assert (p.x == 1);
  assert (p.y == 2);
  assert (p.z == 3);

  // test assignment operator
  testing_output ("assignment operator", 1);
  Position p2 = p;
  assert (p2.x == 1);
  assert (p2.y == 2);
  assert (p2.z == 3);
  
  // equality operator
  testing_output ("equality operator", 1);
  assert (p == p2);

  // inequality operator
  testing_output ("inequality operator", 1);
  assert (!(p != p2));

  // approximately equal
  testing_output ("approximately_equal", 1);
  p2.z = 4;
  assert (!(p.approximately_equal (p2, 0.9)));
  assert (p.approximately_equal (p2, 1));
  p2.y = 3;
  assert (!(p.approximately_equal (p2, 1.4)));
  assert (p.approximately_equal (p2, 1.5));
  p2.x = 2;
  assert (!(p.approximately_equal (p2, 1.7)));
  assert (p.approximately_equal (p2, 1.8));

  // direction to
  testing_output ("direction_to", 1);
  double phi, theta;
  p2.z = p.z;
  p.direction_to (p2, phi, theta);
  assert (phi == M_PI / 4);
  assert (theta == M_PI / 2);

  // distance
  testing_output ("distance", 1);
  p2 = p;
  ++p2.z;
  ++p2.y;
  ++p2.x;
  assert (p.distance_to (p2) == pow (3.0, 0.5));

  // distance_2d
  testing_output ("distance_2d", 1);
  assert (p.distance_to_2d (p2) == pow (2.0, 0.5));

  // test to_string
  testing_output ("to_string", 1);
  assert (p.to_string () == "1,2,3");

  // test from_string
  testing_output ("from_string", 1);
  Position p3 = Position::from_string (p.to_string ());
  assert (p == p3);
  p3 = Position::from_string ("2.3,1.2");
  assert (p3.x == DBL_MAX);
  assert (p3.y == DBL_MAX);
  assert (p3.z == DBL_MAX);
}

// TODO: fill out remaining GPSPosition function tests
void
test_GPSPosition ()
{
  testing_output ("gams::utility::GPSPosition");

  // test constructor
  testing_output ("constructor", 1);
  GPSPosition p (40.442775, -79.940967);
  GPSPosition p2 (40.443412, -79.93954);
  assert (p.latitude () == 40.442775
    && p.longitude () == -79.940967
    && p.altitude () == 0.0);
  assert (p2.latitude () == 40.443412
    && p2.longitude () == -79.93954
    && p2.altitude () == 0.0);

  // test assignment operator
  testing_output ("assignment operator", 1);
  GPSPosition p3 = p;
  assert (p3.latitude () == p.latitude ()
    && p3.longitude () == p.longitude ()
    && p3.altitude () == p.altitude ());

  // test equality operator
  testing_output ("equality operator", 1);
  assert (p3 == p);
  assert (!(p2 == p));

  // test inequality operator
  testing_output ("inequality operator", 1);
  assert (!(p3 != p));
  assert (p2 != p);

  // test approximately equal
  testing_output ("approximately_equal", 1);
  assert (p3.approximately_equal (p, 0.1));
  p3.latitude (40.442776);
  assert (!p3.approximately_equal (p, 0.1));
  assert (p3.approximately_equal (p, 1.0));

  // test direction_to
  testing_output ("direction_to", 1);
  GPSPosition p4 = p;
  p4.latitude (p4.latitude() + 1);
  double phi;
  p.direction_to (p4, phi);
  assert (phi == 0);
  p4 = p;
  p4.longitude (p4.longitude() + 1);
  p.direction_to (p4, phi);
  assert (phi == M_PI / 2);
  p4 = p;
  p4.longitude (p4.longitude() - 1);
  p.direction_to (p4, phi);
  assert (phi == 3 * M_PI / 2);
  p4 = p;
  p4.latitude (p4.latitude() - 1);
  p.direction_to (p4, phi);
  assert (phi == M_PI);
}

// TODO: fill out remaining Region function tests
/*
void
test_Region ()
{
  testing_output ("gams::utility::Region");

  // test constructor
  testing_output ("constructor using to_string", 1);
  vector<GPSPosition> points;
  GPSPosition p (40.443273, -79.939951);
  points.push_back (p);
  p.latitude (40.443116);
  p.longitude (-79.939973);
  points.push_back (p);
  p.latitude (40.443085);
  p.longitude (-79.940313);
  points.push_back (p);
  p.latitude (40.443285);
  p.longitude (-79.940242);
  points.push_back (p);
  Region r (points);
  string expected = "40.443273,-79.939951,0:" \
    "40.443116,-79.939973,0:40.443085,-79.940313,0:" \
    "40.443285,-79.940242,0";
  assert (r.to_string () == expected);

  // test contains
  testing_output ("contains", 1);
  assert (r.contains (p));
  GPSPosition p2 (40.4432, -79.9401);
  assert (r.contains (p2));
  GPSPosition empty;
  assert (!r.contains (empty));

  // bounding box
  testing_output ("get_bounding_box", 1);
  Region bound = r.get_bounding_box ();
  expected = "40.443085,-79.940313,0:" \
    "40.443085,-79.939951,0:40.443285,-79.939951,0:" \
    "40.443285,-79.940313,0";
  assert (r.get_bounding_box ().to_string () == expected);

  // get area
  testing_output ("get_area", 1);
  assert (r.get_area () < 525.8 && r.get_area() > 525.7);
  assert (bound.get_area () < 681.3 && bound.get_area() > 681.2);

  // check to/from container
  madara::knowledge::KnowledgeBase kb;
  r.to_container (kb, "test");
  Region from;
  from.from_container (kb, "test");
  assert (from == r);
  Region nullRegion;
  assert (from != nullRegion);
}

void
test_SearchArea ()
{
  testing_output ("gams::utility::SearchArea");

  // test constructor
  testing_output ("get_convex_hull", 1);
  vector<GPSPosition> points;
  GPSPosition p (40.443237, -79.94057);
  points.push_back (p);
  p.latitude (40.443387);
  p.longitude (-79.94027);
  points.push_back (p);
  p.latitude (40.443187);
  p.longitude (-79.940098);
  points.push_back (p);
  p.latitude (40.443077);
  p.longitude (-79.940398);
  points.push_back (p);
  PrioritizedRegion pr (points, 1);
  SearchArea search (pr);
  Region convex1 = search.get_convex_hull ();

  points.clear ();
  p.latitude (40.443237);
  p.longitude (-79.94047);
  points.push_back (p);
  p.latitude (40.443377);
  p.longitude (-79.94027);
  points.push_back (p);
  p.latitude (40.443337);
  p.longitude (-79.940298);
  points.push_back (p);
#endif // #ifdef _GAMS_OSC_
;
  PrioritizedRegion from;
  from.from_container (kb, "test");
  assert (from == pr);
  from.to_container (kb, "test2");
  pr.from_container (kb, "test2");
  assert (from == pr);
  PrioritizedRegion nullPR;
  assert (nullPR != from);
  SearchArea cont1;
  cont1.add_prioritized_region (pr);
  cont1.add_prioritized_region (nullPR);
  SearchArea cont2;
  cont1.to_container (kb, "sa_test");
  cont2.from_container (kb, "sa_test");
  assert (cont1 == cont2);
  SearchArea nullSA;
  nullSA.to_container (kb, "nullSA");
  assert (cont1 != nullSA);
  cont1.from_container (kb, "nullSA");
  assert (cont1 == nullSA);

  // try to get region from search area
  Region fail_region;
  assert (!fail_region.from_container (kb, "nullSA"));
  assert (!fail_region.from_container (kb, "sa_test"));
}
*/

int
main (int /*argc*/, char ** /*argv*/)
{
  gams::loggers::global_logger->set_level (-1);
  test_Position ();
  test_GPSPosition ();

  #ifdef _GAMS_OSC_
  test_OscUdp();
  #endif // #ifdef _GAMS_OSC_

  //test_Region ();
  //test_SearchArea ();
  return 0;
}
