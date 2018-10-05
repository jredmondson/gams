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

  namespace gp = gams::pose;

  class_<gams::pose::ReferenceFrame>(
      "ReferenceFrame", "Provides reference frame transforms", init<>())

    //   .def("id",
    //       &gams::pose::ReferenceFrame::id,
    //       "Gets the ID string of this frame")

      .def("load_tree",
          static_cast<std::vector<gp::ReferenceFrame> (*)(
              madara::knowledge::KnowledgeBase &,
           const std::vector<std::string>&, 
           uint64_t timestamp,
           const gp::FrameEvalSettings &)>(
            &gams::pose::ReferenceFrame::load_tree),
          "Load ReferenceFrames, by ID, and their common ancestors. Will"
          "interpolate frames to ensure the returned frames all have a common"
          "timestamp")
      .staticmethod("load_tree")

      .def("origin",
          +[](gp::ReferenceFrame& frame) { return frame.origin(); },
          "Returns the origin pose of this frame")

      .def("origin_frame",
          +[](gp::ReferenceFrame& frame) { return frame.origin_frame(); },
          "Returns the parent frame of this frame")

      .def("name",
          &gams::pose::ReferenceFrame::name,
          "Returns a human-readable name for the reference frame type")

      .def("id",
          +[](gp::ReferenceFrame &frame) { return frame.id(); },
          "Returns the ID the frame was saved with")

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

      .def("to_string",
          +[](gp::Pose& pose) { return pose.to_string(); },
          "Creates a string representation of this pose")

      .def("is_set",
          &gams::pose::Pose::is_set,
          "Tests if this is invalid; i.e., any values are INVAL_COORD")

      .def("transform_to",
          &gams::pose::Pose::transform_to,
          "Copy and transform this coordinate to a new reference frame")

      .def("transform_this_to",
          &gams::pose::Pose::transform_this_to,
          "Transform this coordinate, in place, to a new reference frame")
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

    class_<gams::pose::FrameEvalSettings>(
       "FrameEvalSettings", "Settings class for saving/loading reference frames",
        init<>())

        // other constructors here
      .def(init<const gams::pose::FrameEvalSettings&>())
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
