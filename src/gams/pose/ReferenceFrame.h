/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
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
 * @file ReferenceFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_POSE_REFERENCE_FRAME_H_
#define _GAMS_POSE_REFERENCE_FRAME_H_

#include <gams/GAMSExport.h>
#include <gams/CPP11_compat.h>
#include <vector>
#include <string>
#include <stdexcept>

#define GAMS_NO_INL
#include "Pose.h"
#undef GAMS_NO_INL

/**
 * Following statement or block is executed only if the frame `coord` belongs
 * to is of type `type` (i.e., can be dynamic_cast to it). The casted frame
 * is provided as a const pointer in `frame_ptr`, in scope only within the
 * following statement/block.
 **/
#define GAMS_WITH_FRAME_TYPE(coord, type, frame_ptr) \
  if(const type *frame_ptr = dynamic_cast<const type *>(&coord.frame()))

// note: (void)frame_ref silences warnings if frame_ref isn't used in body code

namespace gams
{
  namespace pose
  {
    class ReferenceFrame;
    class LinearVector;
    class AngularVector;

    /**
     * Thrown when a reference frame function is called with a Coordinate
     * type (e.g., Pose, Linear, Angular) that frame does not support.
     **/
    class bad_coord_type : public std::runtime_error
    {
    public:
      /**
       * The only Constructor.
       *
       * @param frame the ReferenceFrame that the coordinate belongs to
       * @param fn_name the name of the function that was called
       **/
      bad_coord_type(const ReferenceFrame &frame, const std::string &fn_name);
      ~bad_coord_type() throw();

      std::string fn_name;
      const ReferenceFrame &frame;
    };

    /**
     * Thrown when an an attempt is made to transform between frames
     * that do not belong to the same frame tree.
     **/
    class unrelated_frames : public std::runtime_error
    {
    public:
      /**
       * The only Constructor.
       *
       * @param from_frame the frame the coordinate belongs to
       * @param to_frame the frame the coordinate is being transformed to
       **/
      unrelated_frames(const ReferenceFrame &from_frame,
        const ReferenceFrame &to_frame);
      ~unrelated_frames() throw();

      const ReferenceFrame &from_frame;
      const ReferenceFrame &to_frame;
    };

    /**
     * Thrown when an attempt is made to transform between two frame types,
     * and there's not transform defined.
     *
     * Note that between two frame types A and B, there are four different
     * kinds of transforms, which must be defined individually:
     *    1) From A as parent to B as its child
     *    2) From A as child to B as its parent
     *    3) From B as parent to A as its child
     *    4) From B as child to A as its parent
     **/
    class undefined_transform : public std::runtime_error
    {
    public:
      /**
       * The only Constructor
       *
       * @param parent_frame of the two involved frames, the parent frame
       * @param child_frame of the two involved frames, the child frame
       * @param is_child_to_parent indicates direction of transformation
       * @param unsupported_angular true if the error was due to orientd
       *   reference frames not being supported for this transformation.
       *   Defaults to false;
       **/
      undefined_transform(const ReferenceFrame &parent_frame,
        const ReferenceFrame &child_frame, bool is_child_to_parent,
        bool unsupported_angular = false);
      ~undefined_transform() throw();

      const ReferenceFrame &parent_frame;
      const ReferenceFrame &child_frame;
      bool is_child_to_parent;
      bool unsupported_angular;
    };

    /**
     * Base class for Reference Frames.
     * Inherit from this class if implementing a new reference frame.
     * Otherwise, do not use directly.
     *
     * If implementing a new reference frame, you will need to modify the
     * transform_to_origin and transform_from_origin methods of the reference
     * frames the new reference frame should be able to transform to and from.
     **/
    class GAMSExport ReferenceFrame
    {
    protected:
      /**
       * Default constructor. No parent frame.
       **/
      ReferenceFrame();

      /**
       * Creates a copy of the origin Pose passed in.
       *
       * With C++11, the copy is stored within this object. Without, it is
       * allocated onto the heap.
       **/
      explicit ReferenceFrame(const Pose &origin);

      /**
       * Uses an existing Pose as origin, and maintains a pointer to it.
       * Changes to it affect this frame, the the pose must outlive this
       * object.
       **/
      explicit ReferenceFrame(const Pose *origin);

      /**
       * Copy constructor
       **/
      ReferenceFrame(const ReferenceFrame &o);

      /**
       * Assignment operator
       **/
      void operator=(const ReferenceFrame &o);

      /**
       * Destructor. Frees the origin, if it was allocated in
       * this frame's constructor, or by a call to the origin setter
       **/
      virtual ~ReferenceFrame();

    public:
      /**
       * Gets the origin of this Frame
       *
       * @return the Pose which is the origin within this frame's parent,
       * or, a Pose within this own frame, with all zeros for coordinates,
       * if this frame has no parent.
       **/
      const Pose &origin() const;

      /**
       * Sets the origin of this frame.
       *
       * @param new_origin the new origin
       * @return the new origin
       **/
      const Pose &origin(const Pose &new_origin);

      /**
       * Gets the parent frame (the one the origin is within). Will be *this
       * if no parent frame.
       **/
      const ReferenceFrame &origin_frame() const;

      /**
       * Bind this frame's origin to a Pose. Changes to that Pose will affect
       * this frame. It must not be deallocated before this Frame is.
       *
       * @param new_origin the new origin, which this frame will be bound to
       **/
      void bind_origin(const Pose *new_origin);

      bool origin_is_external() const { return extern_origin_; }

      /**
       * Equality operator.
       *
       * @param other the frame to compare to.
       * @return true if both frames are the same object (i.e., same address).
       *    Otherwise, frames are not considered equal (returns false).
       **/
      bool operator==(const ReferenceFrame &other) const;

      /**
       * Inequality operator.
       *
       * @param other the frame to compare to.
       * @return false if both frames are the same object (i.e., same address).
       *    Otherwise, frames are not considered equal (returns true).
       **/
      bool operator!=(const ReferenceFrame &other) const;

      /**
       * Returns a human-readable name for the reference frame type
       *
       * @return the name reference frame type (e.g., GPS, Cartesian)
       **/
      std::string name() const;

      /**
       * Normalizes individual doubles representing a linear
       **/
      void normalize_linear(double &x, double &y, double &z) const;

      /**
       * Normalizes individual doubles representing a angular
       **/
      void normalize_angular(double &rx, double &ry, double &rz) const;

      /**
       * Normalizes individual doubles representing a pose
       **/
      void normalize_pose(double &x, double &y, double &z,
                          double &rx, double &ry, double &rz) const;

    protected:
      /**
       * Override to return a human-readable name for new reference frame types
       *
       * @return the name
       **/
      virtual std::string get_name() const = 0;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       **/
      virtual void transform_linear_to_origin(
                      double &x, double &y, double &z) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       **/
      virtual void transform_linear_from_origin(
                      double &x, double &y, double &z) const;

      /**
       * Override for new coordinate systems that can require normalization of
       * coordinates, e.g., angle-based systems. NOP by default.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       **/
      virtual void do_normalize_linear(
                      double &x, double &y, double &z) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param x1   x coordinate of first linear
       * @param y1   y coordinate of first linear
       * @param z1   z coordinate of first linear
       * @param x2   x coordinate of other linear
       * @param y2   y coordinate of other linear
       * @param z2   z coordinate of other linear
       * @return distance in meters from loc1 to loc2
       **/
      virtual double calc_distance(
                      double x1, double y1, double z1,
                      double x2, double y2, double z2) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_angular_to_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_angular_from_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_to_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_from_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Override for new coordinate systems. By default, throws bad_coord_type.
       *
       * @param rx1   x axis of first angular
       * @param ry1   y axis of first angular
       * @param rz1   z axis of first angular
       * @param rx2   x axis of second angular
       * @param ry2   y axis of second angular
       * @param rz2   z axis of second angular
       * @return angularal distance in degrees from rot1 to rot2
       **/
      virtual double calc_angle(
                      double rx1, double ry1, double rz1,
                      double rx2, double ry2, double rz2) const;

      /**
       * Override for new coordinate systems that can require normalization of
       * coordinates, e.g., angl-based systems. NOP by default.
       *
       * @param rx   x axis of angular
       * @param ry   y axis of angular
       * @param rz   z axis of angular
       **/
      virtual void do_normalize_angular(
                      double &rx, double &ry, double &rz) const;

      /**
       * Override for new coordinate systems that can require normalization of
       * coordinates, e.g., angle-based systems. NOP by default.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx   x axis of angular
       * @param ry   y axis of angular
       * @param rz   z axis of angular
       **/
      virtual void do_normalize_pose(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Normalizes any angular values in coord according to the conventions
       * of the frame it belongs to. This is called automatically by any methods
       * that require normalized angles.
       *
       * If adding a new composite coordinate type, specialize this template
       * function accordingly.
       * 
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param coord the Coordinate to normalize
       **/
      template<typename CoordType>
      static void normalize(CoordType &coord);

      /**
       * Transform input coordinates into their common parent. If no common
       * parent exists, throws, unrelated_frames exception
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in1 the frame to transform from
       * @param in2 the frame to transform into
       * @return the common frame the coordinates were transformed to.
       *
       * @throws unrelated_frame if no common parent.
       **/
      template<typename CoordType>
      static const ReferenceFrame &common_parent_transform(
            CoordType &in1, CoordType &in2);

      /**
       * Transform coordinate from its current from, to the specified frame
       * This transformation is in-place (modifies the in parameters.
       * Called by the transform_to member function of Coordinates
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in the Coordinate to transform. This object may be modified.
       * @param to_frame the frame to transform into
       *
       * @throws unrelated_frame if no common parent.
       **/
      template<typename CoordType>
      static void transform(CoordType &in, const ReferenceFrame &to_frame);

      /**
       * Calculate distances between coordinates. If they have different
       * frames, first transform to their lowest common parent
       * Called by the distance_to member function of Coordinates
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param coord1 The coordinate to measure from
       * @param coord2 The coordinate to measure to
       * @return the distance, in meters, between the coordinates
       **/
      template<typename CoordType>
      static double distance(const CoordType &coord1, const CoordType &coord2);

      /**
       * Transforms a coordinate into a frame's origin frame.
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in the Coordinate to transform. This object may be modified.
       **/
      template<typename CoordType>
      static void transform_to_origin(CoordType &in);

      /**
       * Transforms a coordinate into a frame from its origin frame.
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in the Coordinate to transform. This object may be modified.
       * @param to_frame the frame to transform into. Must be origin of in.
       **/
      template<typename CoordType>
      static void transform_from_origin(CoordType &in,
          const ReferenceFrame &to_frame);

      /**
       * Calculates distance between coordinates within a specific frame
       * For coordinate types which can be expressed in terms of existing
       * coordinate types, specialize this function to express this fact
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param coord1 The coordinate to measure from
       * @param coord2 The coordinate to measure to
       * @return the distance, in meters, between the coordinates
       *
       * @pre coord1 and coord2 must have same frame, passed as "frame"
       **/
      template<typename CoordType>
      static double calc_difference(const CoordType &coord1,
                                    const CoordType &coord2);

      /**
       * Helper function to find the common frame between two frames.
       *
       * @param from the initial frame
       * @param to the target frame
       * @param to_stack if not nullptr, the frames needed to go from base to
       *  target frame will be pushed to pointed to vector
       **/
      static const ReferenceFrame *find_common_frame(
          const ReferenceFrame *from,
          const ReferenceFrame *to,
          std::vector<const ReferenceFrame *> *to_stack = nullptr);

      /**
       * Transform into another frame, if coordinates are not directly related.
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in the coordinate to transform (in-place)
       * @param to_frame the frame to transform into
       *
       * @throws unrelated_frame if no common parent.
       **/
      template<typename CoordType>
      inline static void transform_other(
          CoordType &in, const ReferenceFrame &to_frame);

      /**
       * Transform into common parent, if coordinates are not directly related.
       *
       * @tparam CoordType the type of Coordinate (e.g., Pose, Linear)
       * @param in1 the coordinate to transform (in place)
       * @param in2 the other coordinate to transform (in place)
       * @return the common frame the coordinates were transformed to.
       *
       * @throws unrelated_frame if no common parent.
       **/
      template<typename CoordType>
      static const ReferenceFrame &common_parent_transform_other(
          CoordType &in1, CoordType &in2);

      template<typename CoordType>
      friend class Coordinate;

    protected:
#ifdef CPP11
      union
      {
        const Pose *ptr_origin_;
        Pose origin_;
      };
#else
      const Pose *origin_;
#endif
      bool extern_origin_;

    public:
    };

    /**
     * For internal use.
     *
     * Provides implementation of angular and pose transforms for frames
     * where angular transformation is independant of linear. This applies
     * to, for example, Cartesian and GPS frames, but not UTM frames.
     * Inherit from this to use this implementation.
     **/
    class GAMSExport SimpleRotateFrame : public ReferenceFrame
    {
    protected:
      /**
       * Default constructor. No parent frame.
       **/
      SimpleRotateFrame();

      /**
       * Creates a copy of the origin Pose passed in.
       *
       * @param origin the frame's origin
       **/
      explicit SimpleRotateFrame(const Pose &origin);

      /**
       * Uses an existing Pose as origin, and maintains
       * a pointer to it. Changes to it affect this frame
       *
       * @param origin the origin to bind to
       **/
      explicit SimpleRotateFrame(Pose *origin);

    protected:
      /**
       * Rotates a LinearVector according to a AngularVector
       *
       * @param x   the x coordinate to orient (in-place)
       * @param y   the y coordinate to orient (in-place)
       * @param z   the z coordinate to orient (in-place)
       * @param rot the angular to apply, axis-angle notation
       * @param reverse if true, apply angular in opposite direction
       **/
      void orient_linear_vec(
          double &x, double &y, double &z,
          const AngularVector &rot,
          bool reverse = false) const;

    private:
      /**
       * Transform AngularVector in-place into its origin frame from this frame
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_angular_to_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform AngularVector in-place from its origin frame
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_angular_from_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform pose in-place into its origin frame from this frame.
       * Simply applies linear and angular transforms independantly
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_to_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform pose in-place from its origin frame
       * Simply applies linear and angular transforms independantly
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_from_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Calculates smallest angle between two AngularVectors
       *
       * @param rx1  the starting angular on x axis
       * @param ry1  the starting angular on y axis
       * @param rz1  the starting angular on z axis
       * @param rx2 the ending rotatation on x axis
       * @param ry2  the ending angular on y axis
       * @param rz2  the ending angular on z axis
       * @return the difference in radians
       **/
      virtual double calc_angle(
                      double rx1, double ry1, double rz1,
                      double rx2, double ry2, double rz2) const;
    };

    GAMSExport const ReferenceFrame &default_frame (void);
  }
}

#include "ReferenceFrame.inl"

#endif
