#ifndef _GAMS_PYTHON_PORT_GAMS_POSE_CPP_
#define _GAMS_PYTHON_PORT_GAMS_POSE_CPP_

#include <boost/python/detail/wrap_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/import.hpp>
#include <boost/python/enum.hpp>

#include "gams/pose/Pose.h"
#include "gams/pose/Orientation.h"
#include "gams/pose/Position.h"
#include "gams/pose/Coordinate.h"
#include "gams/pose/ReferenceFrame.h"

#include "FunctionDefaults.h"
#include "GamsPose.h"

/**
 * @file GamsPose.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains Boost.Python mappings for gams.pose module
 **/

using namespace boost::python;

class Pose_NS
{
};

void define_pose(void)
{
  object pose = object(handle<>(PyModule_New("gams.pose")));

  pose.attr("__file__") = "<synthetic>";
  scope().attr("pose") = pose;
  pose.attr("__doc__") = "Provides access to poses and reference frames";

  // this was the missing piece: sys.modules['modA.modB']=modB
  extract<dict>(getattr(import("sys"), "modules"))()["gams.pose"] =
      pose;

  scope Pose = pose;

  /********************************************************
   * Frame definitions
   ********************************************************/

  class_<gams::pose::ReferenceFrame>(
      "ReferenceFrame", "Provides reference frame transforms", init<>())

    //   .def("id",
    //       &gams::pose::ReferenceFrame::id,
    //       "Gets the ID string of this frame")

      .def("name",
          &gams::pose::ReferenceFrame::name,
          "Returns a human-readable name for the reference frame type")

      .def("timestamp",
          static_cast<uint64_t (gams::pose::ReferenceFrame::*)(
              ) const> (
              &gams::pose::ReferenceFrame::timestamp),
          "Tests whether the frame is valid")

      .def("valid",
          &gams::pose::ReferenceFrame::valid,
          "Tests whether the frame is valid")
      ;

  /********************************************************
   * Fixed vector definitions
   ********************************************************/

  class_<gams::pose::Pose>(
      "Pose", "A combination of position and orientation", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Pose::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Pose::from_container),
          "Imports the pose from a MADARA container NativeDoubleVector")

      .def("from_container",
          static_cast<void (gams::pose::Pose::*)(
              const std::vector <double> &)>(
              &gams::pose::Pose::from_container),
          "Imports the pose from a STL vector of doubles. "
          "See gams.from_pydoubles for useful conversion.")

      .def("is_set",
          &gams::pose::Pose::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::Orientation>(
      "Orientation", "The orientation vector of an agent", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Orientation::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Orientation::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::Orientation::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::Position>(
      "Position", "The position of an agent", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Position::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Position::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::Position::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  /********************************************************
   * Free vector definitions
   ********************************************************/

  class_<gams::pose::Displacement>(
      "Displacement", "Distance between positions", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Displacement::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Displacement::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::Displacement::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::Velocity>(
      "Velocity", "Speed in given direction", init<>())


      .def("from_container",
          static_cast<void (gams::pose::Velocity::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Velocity::from_container),
          "Imports from a MADARA container NativeDoubleVector")


      .def("is_set",
          &gams::pose::Velocity::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::Acceleration>(
      "Acceleration", "Rate of change of speed in a given direction", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Acceleration::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Acceleration::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::Acceleration::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  /********************************************************
   * Origin-less vector definitions
   ********************************************************/

  class_<gams::pose::Rotation>(
      "Rotation", "Distance between positions", init<>())

      .def("from_container",
          static_cast<void (gams::pose::Rotation::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::Rotation::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::Rotation::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::AngularVelocity>(
      "AngularVelocity", "Speed in given direction (no origin)", init<>())

      .def("from_container",
          static_cast<void (gams::pose::AngularVelocity::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::AngularVelocity::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::AngularVelocity::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

  class_<gams::pose::AngularAcceleration>(
      "AngularAcceleration",
      "Rate of change of speed in a given direction (no origin)",
        init<>())

      .def("from_container",
          static_cast<void (gams::pose::AngularAcceleration::*)(
              const madara::knowledge::containers::NativeDoubleVector &)>(
              &gams::pose::AngularAcceleration::from_container),
          "Imports from a MADARA container NativeDoubleVector")

      .def("is_set",
          &gams::pose::AngularAcceleration::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")
      ;

}

#endif  // _GAMS_PYTHON_PORT_GAMS_POSE_CPP_
