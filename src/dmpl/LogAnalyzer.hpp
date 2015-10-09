/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:

 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following acknowledgments
 * and disclaimers.

 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.

 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 * Engineering Institute" shall not be used to endorse or promote
 * products derived from this software without prior written
 * permission. For written permission, please contact
 * permission@sei.cmu.edu.

 * 4. Products derived from this software may not be called "SEI" nor
 * may "SEI" appear in their names without prior written permission of
 * permission@sei.cmu.edu.

 * 5. Redistributions of any form whatsoever must retain the following
 * acknowledgment:

 * This material is based upon work funded and supported by the
 * Department of Defense under Contract No. FA8721-05-C-0003 with
 * Carnegie Mellon University for the operation of the Software
 * Engineering Institute, a federally funded research and development
 * center.

 * Any opinions, findings and conclusions or recommendations expressed
 * in this material are those of the author(s) and do not necessarily
 * reflect the views of the United States Department of Defense.

 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE
 * ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS"
 * BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT
 * LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY,
 * EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE
 * MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH
 * RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT
 * INFRINGEMENT.

 * This material has been approved for public release and unlimited
 * distribution.

 * DM-0002494
**/

#ifndef DMPL_LOG_ANALYZER
#define DMPL_LOG_ANALYZER

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/KnowledgeRecord.h"

namespace engine = madara::knowledge;
using madara::KnowledgeRecord;

namespace dmpl
{
  class LogAnalyzer
  {
  protected:
    istream &in;
    engine::KnowledgeBase &kbase;
    long active_frame;
    long cur_frame;

    struct Row
    {
      int frame;
      double time;
      int node;
      std::string var;
      std::string madara_name;
      KnowledgeRecord val;
    };

    Row cur_row;

    // Taken from http://stackoverflow.com/a/1120224, licensed under CC-SA
    std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
    {
        std::vector<std::string> result;
        std::string line;

        if(!std::getline(str,line))
          return result;

        std::stringstream lineStream(line);
        std::string cell;

        while(std::getline(lineStream,cell,','))
        {
            result.push_back(cell);
        }
        return result;
    }

    template<class T>
    T stream_cast(const std::string &s)
    {
      T ret;
      std::stringstream ss(s);
      ss >> ret;
      return ret;
    }

    bool get_next_row()
    {
      std::vector<std::string> row = getNextLineAndSplitIntoTokens(in);
      if(row.size() == 0)
      {
        return false;
      }
      cur_row.frame = stream_cast<int>(row[0]);
      cur_row.time = stream_cast<double>(row[1]);
      cur_row.node = stream_cast<int>(row[2]);
      cur_row.var = row[3];
      cur_row.madara_name = cur_row.var + "." + row[2];
      if(row[4].find('.') == std::string::npos)
        cur_row.val.set_value(stream_cast<KnowledgeRecord::Integer>(row[4]));
      else
        cur_row.val.set_value(stream_cast<double>(row[4]));
      return true;
    }

    void get_header()
    {
      std::vector<std::string> row = getNextLineAndSplitIntoTokens(in);
    }

    void update_knowledge(const Row &row)
    {
      //std::cout << "Setting " << row.madara_name << " to " << row.val << std::endl;
      kbase.get_context().set(row.madara_name, row.val);
    }
  public:

    LogAnalyzer(istream &csv_stream, engine::KnowledgeBase &knowledge)
      : in(csv_stream), kbase(knowledge), active_frame(0)
    {
      get_header();
      get_next_row();
      cur_frame = cur_row.frame;
    }

    bool next_step()
    {
      for(;;)
      {
        cur_frame = cur_row.frame;
        //std::cout << std::fixed;
        //std::cout << cur_row.frame << " " << cur_row.time << " " << cur_row.var << "@" << cur_row.node << ": "
                  //<< cur_row.val << "  (" << cur_row.val.type() << ")" << std::endl;
        //std::cout.unsetf(ios_base::floatfield);

        update_knowledge(cur_row);
        if(!get_next_row())
        {
          //std::cout << "END OF FRAME " << active_frame << std::endl;
          //std::cout << "END OF FILE" << std::endl;
          return false;
        }
        if(cur_row.frame > active_frame)
        {
          //std::cout << "END OF FRAME " << active_frame << std::endl;
          active_frame = cur_row.frame;
          return true;
        }
        // else, loop for next entry in current frame
      }

    }

    long get_cur_frame() { return cur_frame; }
  };
}
#endif
