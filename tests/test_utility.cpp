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

#include "gams/utility/Position.h"
#include "gams/utility/GPSPosition.h"
#include "gams/pose/Region.h"
#include "gams/pose/PrioritizedRegion.h"
#include "gams/pose/SearchArea.h"

#include "gams/loggers/GlobalLogger.h"

#include "gtest/gtest.h"

using gams::utility::GPSPosition;
using gams::utility::Position;
using gams::pose::PrioritizedRegion;
using gams::pose::Region;
using gams::pose::SearchArea;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// TODO: fill out remaining Position function tests
TEST(TestUtility, TestPosition)
{

  // test constructor
  const Position p (1,2,3);
  EXPECT_EQ(p.x, 1);
  EXPECT_EQ(p.y, 2);
  EXPECT_EQ(p.z, 3);

  // test assignment operator
  Position p2 = p;
  EXPECT_EQ(p2.x, 1);
  EXPECT_EQ(p2.y, 2);
  EXPECT_EQ(p2.z, 3);
  
  // equality operator
  EXPECT_TRUE(p == p2);

  // inequality operator
  EXPECT_TRUE(!(p != p2));

  // approximately equal
  p2.z = 4;
  EXPECT_TRUE(!(p.approximately_equal (p2, 0.9)));
  EXPECT_TRUE(p.approximately_equal (p2, 1));
  p2.y = 3;
  EXPECT_TRUE(!(p.approximately_equal (p2, 1.4)));
  EXPECT_TRUE(p.approximately_equal (p2, 1.5));
  p2.x = 2;
  EXPECT_TRUE(!(p.approximately_equal (p2, 1.7)));
  EXPECT_TRUE(p.approximately_equal (p2, 1.8));

  // direction to
  double phi, theta;
  p2.z = p.z;
  p.direction_to (p2, phi, theta);
  EXPECT_FLOAT_EQ(phi, M_PI / 4);
  EXPECT_FLOAT_EQ(theta, M_PI / 2);

  // distance
  p2 = p;
  ++p2.z;
  ++p2.y;
  ++p2.x;
  EXPECT_FLOAT_EQ(p.distance_to (p2), pow (3.0, 0.5));

  // distance_2d
  EXPECT_FLOAT_EQ (p.distance_to_2d (p2), pow (2.0, 0.5));

  // test to_string
  EXPECT_EQ(p.to_string (), "1,2,3");

  // test from_string
  Position p3 = Position::from_string (p.to_string ());
  EXPECT_EQ(p, p3);
  p3 = Position::from_string ("2.3,1.2");
  EXPECT_DOUBLE_EQ(p3.x, DBL_MAX);
  EXPECT_DOUBLE_EQ(p3.y, DBL_MAX);
  EXPECT_DOUBLE_EQ(p3.z, DBL_MAX);
}

// TODO: fill out remaining GPSPosition function tests
TEST(TestUtility, TestGPSPosition)
{
  // test constructor
  GPSPosition p (40.442775, -79.940967);
  GPSPosition p2 (40.443412, -79.93954);
  EXPECT_FLOAT_EQ(p.latitude(), 40.442775);
  EXPECT_FLOAT_EQ(p.longitude(), -79.940967);
  EXPECT_FLOAT_EQ(p.altitude(), 0.0);
  EXPECT_FLOAT_EQ(p2.latitude(), 40.443412);
  EXPECT_FLOAT_EQ(p2.longitude (), -79.93954);
  EXPECT_FLOAT_EQ(p2.altitude (), 0.0);

  // test assignment operator
  GPSPosition p3 = p;
  EXPECT_EQ(p3.latitude(), p.latitude());
  EXPECT_EQ(p3.longitude(), p.longitude());
  EXPECT_EQ(p3.altitude(), p.altitude());

  // test equality operator
  EXPECT_TRUE(p3 == p);
  EXPECT_TRUE(!(p2 == p));

  // test inequality operator
  EXPECT_TRUE(!(p3 != p));
  EXPECT_TRUE(p2 != p);

  // test approximately equal
  EXPECT_TRUE(p3.approximately_equal (p, 0.1));
  p3.latitude (40.442776);
  EXPECT_TRUE(!p3.approximately_equal (p, 0.1));
  EXPECT_TRUE(p3.approximately_equal (p, 1.0));

  // test direction_to
  GPSPosition p4 = p;
  p4.latitude (p4.latitude() + 1);
  double phi;
  p.direction_to (p4, phi);
  EXPECT_DOUBLE_EQ(phi, 0);
  p4 = p;
  p4.longitude (p4.longitude() + 1);
  p.direction_to (p4, phi);
  EXPECT_DOUBLE_EQ(phi, M_PI / 2);
  p4 = p;
  p4.longitude (p4.longitude() - 1);
  p.direction_to (p4, phi);
  EXPECT_DOUBLE_EQ(phi, 3 * M_PI / 2);
  p4 = p;
  p4.latitude (p4.latitude() - 1);
  p.direction_to (p4, phi);
  EXPECT_DOUBLE_EQ(phi, M_PI);
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
  PrioritizedRegion pr2 (points, 5);
  search.add_prioritized_region (pr2);
  assert (search.get_convex_hull () == convex1);

  // test to/from container
  madara::knowledge::KnowledgeBase kb;
  pr.to_container (kb, "test");
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
main (int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
