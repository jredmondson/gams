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
 * @file Base_Frame.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#include <gams/utility/Base_Frame.h>

namespace gams
{
  namespace utility
  {
    // TODO implement O(height) algo instead of O(height ^ 2)
    const Base_Frame *Base_Frame::find_common_frame(
      const Base_Frame *from, const Base_Frame *to,
      std::vector<const Base_Frame *> *to_stack)
    {
      const Base_Frame *cur_to = to;
      for(;;)
      {
        const Base_Frame *cur_from = from;
        for(;;)
        {
          if(cur_to == cur_from)
          {
            return cur_to;
          }
          const Base_Frame *next_cur_from = &cur_from->origin().frame();
          if(cur_from == next_cur_from)
            break;
          cur_from = next_cur_from;
        }
        if(to_stack)
          to_stack->push_back(cur_to);
        const Base_Frame *next_cur_to = &cur_to->origin().frame();
        if(cur_to == next_cur_to)
          break;
        cur_to = next_cur_to;
      }
      return NULL;
    }

    void Basic_Rotational_Frame::transform_to_origin(Rotation_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin(), Basic_Rotational_Frame, frame)
      {
        Quaternion in_quat(in);
        Quaternion origin_quat(static_cast<const Rotation_Base &>(origin()));
        in_quat *= origin_quat;
        in_quat.to_rotation_vector(in);
        return;
      }
      throw undefined_transform(*this, origin().frame(), true);
    }

    void Basic_Rotational_Frame::transform_from_origin(Rotation_Base &in) const
    {
      GAMS_WITH_FRAME_TYPE(origin(), Basic_Rotational_Frame, frame)
      {
        Quaternion in_quat(in);
        Quaternion origin_quat(static_cast<const Rotation_Base &>(origin()));
        origin_quat.conjugate();
        in_quat *= origin_quat;
        in_quat.to_rotation_vector(in);
        return;
      }
      throw undefined_transform(*this, origin().frame(), false);
    }
  }
}
