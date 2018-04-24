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
#include <gams/pose/CartesianFrame.h>
#include <gams/pose/Linear.h>
#include <gams/pose/Angular.h>
#include <gams/pose/Quaternion.h>

namespace gams
{
  namespace pose
  {
    // TODO implement O(height) algo instead of O(height ^ 2)
    const ReferenceFrame *find_common_frame(
      const ReferenceFrame *from, const ReferenceFrame *to,
      std::vector<const ReferenceFrame *> *to_stack)
    {
      const ReferenceFrame *cur_to = to;
      for(;;)
      {
        const ReferenceFrame *cur_from = from;
        for(;;)
        {
          if(*cur_to == *cur_from)
          {
            return cur_to;
          }
          cur_from = &cur_from->origin().frame();
          if(!cur_from->valid())
            break;
        }
        if(to_stack)
          to_stack->push_back(cur_to);
        cur_to = &cur_to->origin().frame();
        if(!cur_to->valid())
          break;
      }
      return nullptr;
    }

    namespace simple_rotate {
      void orient_linear_vec(
            double &x, double &y, double &z,
            double rx, double ry, double rz,
            bool reverse)
      {
        if(rx == 0 && ry == 0 && rz == 0)
          return;

        Quaternion locq(x, y, z, 0);
        Quaternion rotq(rx, ry, rz);

        if(reverse)
          rotq.conjugate();

        locq.orient_by(rotq);
        locq.to_linear_vector(x, y, z);
      }

      void transform_angular_to_origin(
                        const ReferenceFrameType *,
                        const ReferenceFrameType *,
                        double orx, double ory, double orz,
                        double &rx, double &ry, double &rz)
      {
        Quaternion in_quat(rx, ry, rz);
        Quaternion origin_quat(orx, ory, orz);
        in_quat *= origin_quat;
        in_quat.to_angular_vector(rx, ry, rz);
      }

      void transform_angular_from_origin(
                        const ReferenceFrameType *,
                        const ReferenceFrameType *,
                        double orx, double ory, double orz,
                        double &rx, double &ry, double &rz)
      {
        Quaternion in_quat(rx, ry, rz);
        Quaternion origin_quat(orx, ory, orz);
        origin_quat.conjugate();
        in_quat *= origin_quat;
        in_quat.to_angular_vector(rx, ry, rz);
      }

      void transform_pose_to_origin(
                      const ReferenceFrameType *other,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz)
      {
        self->transform_linear_to_origin(other, self,
                          ox, oy, oz,
                          orx, ory,
                          orz, x, y, z);
        self->transform_angular_to_origin(other, self,
                          orx, ory, orz,
                          rx, ry, rz);
      }

      void transform_pose_from_origin(
                      const ReferenceFrameType *other,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz)
      {
        self->transform_linear_from_origin(other, self,
                          ox, oy, oz,
                          orx, ory,
                          orz, x, y, z);
        self->transform_angular_from_origin(other, self,
                          orx, ory, orz,
                          rx, ry, rz);
      }

      double calc_angle(
                  const ReferenceFrameType *,
                  double rx1, double ry1, double rz1,
                  double rx2, double ry2, double rz2)
      {
        Quaternion quat1(rx1, ry1, rz1);
        Quaternion quat2(ry2, ry2, rz2);
        return quat1.angle_to(quat2);
      }
    }

    const ReferenceFrame &default_frame (void)
    {
      static ReferenceFrame frame{"default_frame", Pose(ReferenceFrame(), 0, 0)};
      return frame;
    }
  }
}
