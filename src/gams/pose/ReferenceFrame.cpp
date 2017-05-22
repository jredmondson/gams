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
 * @file ReferenceFrame.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#include <gams/pose/ReferenceFrame.h>
#include <gams/pose/Position.h>
#include <gams/pose/Orientation.h>
#include <gams/pose/Quaternion.h>

namespace gams
{
  namespace pose
  {
    // TODO implement O(height) algo instead of O(height ^ 2)
    const ReferenceFrame *ReferenceFrame::find_common_frame(
      const ReferenceFrame *from, const ReferenceFrame *to,
      std::vector<const ReferenceFrame *> *to_stack)
    {
      const ReferenceFrame *cur_to = to;
      for(;;)
      {
        const ReferenceFrame *cur_from = from;
        for(;;)
        {
          if(cur_to == cur_from)
          {
            return cur_to;
          }
          const ReferenceFrame *next_cur_from = &cur_from->origin().frame();
          if(cur_from == next_cur_from)
            break;
          cur_from = next_cur_from;
        }
        if(to_stack)
          to_stack->push_back(cur_to);
        const ReferenceFrame *next_cur_to = &cur_to->origin().frame();
        if(cur_to == next_cur_to)
          break;
        cur_to = next_cur_to;
      }
      return nullptr;
    }

    void ReferenceFrame::transform_position_to_origin(
                                double &, double &, double &) const
    {
      throw bad_coord_type<Position>(*this, "transform_position_to_origin");
    }

    void ReferenceFrame::transform_position_from_origin(
                                double &, double &, double &) const
    {
      throw bad_coord_type<Position>(*this, "transform_position_from_origin");
    }

    void ReferenceFrame::do_normalize_position(
                double &, double &, double &) const {}

    double ReferenceFrame::calc_distance(
                double, double, double, double, double, double) const
    {
      throw bad_coord_type<Position>(*this, "calc_distance");
    }

    void ReferenceFrame::transform_orientation_to_origin(
                                double &, double &, double &) const
    {
      throw bad_coord_type<Orientation>(*this, "transform_orientation_to_origin");
    }

    void ReferenceFrame::transform_orientation_from_origin(
                                double &, double &, double &) const
    {
      throw bad_coord_type<Orientation>(*this, "transform_orientation_from_origin");
    }

    void ReferenceFrame::transform_pose_to_origin(
                    double &x, double &y, double &z,
                    double &rx, double &ry, double &rz) const
    {
      throw bad_coord_type<Orientation>(*this, "transform_pose_to_origin");
    }

    void ReferenceFrame::transform_pose_from_origin(
                    double &x, double &y, double &z,
                    double &rx, double &ry, double &rz) const
    {
      throw bad_coord_type<Orientation>(*this, "transform_pose_from_origin");
    }

    double ReferenceFrame::calc_angle(
                double, double, double, double, double, double) const
    {
      throw bad_coord_type<Orientation>(*this, "calc_angle");
    }

    void ReferenceFrame::do_normalize_orientation(
                double &, double &, double &) const {}

    void ReferenceFrame::do_normalize_pose(
                    double &x, double &y, double &z,
                    double &rx, double &ry, double &rz) const
    {
      do_normalize_position(x, y, z);
      do_normalize_orientation(rx, ry, rz);
    }

    void SimpleRotateFrame::orient_position_vec(
          double &x, double &y, double &z,
          const OrientationVector &rot, bool reverse) const
    {
      if(rot.is_zero())
        return;

      Quaternion locq(x, y, z, 0);
      Quaternion rotq(rot);

      if(reverse)
        rotq.conjugate();

      locq.orient_by(rotq);
      locq.to_position_vector(x, y, z);
    }

    void SimpleRotateFrame::transform_orientation_to_origin(
                                double &rx, double &ry, double &rz) const
    {
      Quaternion in_quat(rx, ry, rz);
      Quaternion origin_quat(origin().as_orientation_vec());
      in_quat *= origin_quat;
      in_quat.to_orientation_vector(rx, ry, rz);
    }

    void SimpleRotateFrame::transform_orientation_from_origin(
                                double &rx, double &ry, double &rz) const
    {
      Quaternion in_quat(rx, ry, rz);
      Quaternion origin_quat(origin().as_orientation_vec());
      origin_quat.conjugate();
      in_quat *= origin_quat;
      in_quat.to_orientation_vector(rx, ry, rz);
    }

    void SimpleRotateFrame::transform_pose_to_origin(
                    double &x, double &y, double &z,
                    double &rx, double &ry, double &rz) const
    {
      transform_position_to_origin(x, y, z);
      transform_orientation_to_origin(rx, ry, rz);
    }

    void SimpleRotateFrame::transform_pose_from_origin(
                    double &x, double &y, double &z,
                    double &rx, double &ry, double &rz) const
    {
      transform_position_from_origin(x, y, z);
      transform_orientation_from_origin(rx, ry, rz);
    }

    inline double SimpleRotateFrame::calc_angle(
                double rx1, double ry1, double rz1,
                double rx2, double ry2, double rz2) const
    {
      Quaternion quat1(rx1, ry1, rz1);
      Quaternion quat2(ry2, ry2, rz2);
      return quat1.angle_to(quat2);
    }

  }
}
