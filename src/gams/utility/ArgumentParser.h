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
 * @file ArgumentParser.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the ArgumentParser class
 **/

#ifndef _GAMS_UTILITY_ARGUMENT_PARSER_H_
#define _GAMS_UTILITY_ARGUMENT_PARSER_H_

#include <iostream>
#include <string>
#include <gams/CPP11_compat.h>
#include <madara/utility/Utility.h>
#include <madara/knowledge/KnowledgeBase.h>
#include <iterator>

namespace gams
{
  namespace utility
  {
    class ArgumentParser
    {
    public:
      ArgumentParser(
          const std::string &prefix,
          const madara::knowledge::KnowledgeBase &kbase,
          const madara::knowledge::KnowledgeVector &kvec)
        : prefix_(prefix), kmap_(kbase.to_map(prefix)), kvec_(&kvec) {}

      ArgumentParser(
          const std::string &prefix,
          const madara::knowledge::KnowledgeBase &kbase)
        : prefix_(prefix), kmap_(kbase.to_map(prefix)), kvec_(NULL) {}

      ArgumentParser(const madara::knowledge::KnowledgeVector &kvec)
        : prefix_(""), kmap_(), kvec_(&kvec) {}

    private:
      template<class map_iterator, class vec_iterator>
      class my_iterator : public std::iterator<
        std::input_iterator_tag, typename map_iterator::value_type >
      {
      public:
        //typedef std::iterator<
           //std::input_iterator_tag, typename map_iterator::value_type >
             //base_iterator;
        typedef typename my_iterator::value_type value_type;
        //typedef typename base_iterator::value_type value_type;
      private:
        my_iterator() : parent_(NULL), cache_valid_(0) {}

        my_iterator(const ArgumentParser &parent)
          : parent_(&parent), map_i_(parent_->kmap_.begin()), cache_valid_(0)
        {
          if(parent_->kvec_ != NULL)
            vec_i_ = parent_->kvec_->begin();
          skip_invalid();
        }

#ifdef CPP11
        alignas(value_type)
#endif
        mutable char cache_[sizeof(value_type)];
        mutable bool cache_valid_;

        const ArgumentParser *parent_;
        map_iterator map_i_;
        vec_iterator vec_i_;

      public:
        bool map_at_end() const
        {
          return map_i_ == parent_->kmap_.end();
        }

        bool vec_at_end() const
        {
          return (parent_->kvec_ == NULL ||
                  vec_i_ >= parent_->kvec_->end());
        }

        bool at_end() const
        {
          return parent_ == NULL ||
                 (map_at_end() && vec_at_end());
        }

        bool operator==(const my_iterator &o) const
        {
          bool ae1 = at_end(), ae2 = o.at_end();
          if(ae1 || ae2)
            return ae1 && ae2;
          else
            return parent_ == o.parent_ &&
                   map_i_ == o.map_i_ &&
                   (parent_->kvec_ == NULL ||
                     vec_i_ == o.vec_i_);
        }

        bool operator!=(const my_iterator &o) const
        {
          return !(*this == o);
        }

      private:
        std::string trim_name(const std::string &name) const
        {
          return name.substr(parent_->prefix_.size());
        }

      public:
        std::string name() const
        {
          if(!map_at_end())
            return trim_name(map_i_->first);
          else
            return vec_i_->to_string();
        }

        typename value_type::second_type value() const
        {
          if(!map_at_end())
            return map_i_->second;
          else
            return *(vec_i_ + 1);
        }

      private:
        bool map_name_is_valid() const
        {
          ssize_t psz = parent_->prefix_.size();
          const std::string &n = map_i_->first;
          if(n.compare(psz, std::string::npos, "size") == 0)
            return false;
          char c = n[psz];
          if(c >= '0' && c <= '9')
            return false;
          return true;
        }

        void skip_invalid()
        {
          while(!map_at_end() && !map_name_is_valid())
          {
            ++map_i_;
          }
        }

        void clear_cache()
        {
          if(cache_valid_)
          {
            reinterpret_cast<const value_type *>(&(cache_[0]))->~value_type();
            cache_valid_ = false;
          }
        }

      public:
        void next()
        {
          if(!map_at_end())
          {
            ++map_i_;
            skip_invalid();
          }
          else
            vec_i_ += 2;
          clear_cache();
        }

        const value_type &operator*() const
        {
          if(cache_valid_)
            return *reinterpret_cast<const value_type *>(&(cache_[0]));
          const value_type *ret = new((void*)cache_) value_type(name(),value());
          cache_valid_ = true;
          return *ret;
        }

        my_iterator &operator++()
        {
          next();
          return *this;
        }

        my_iterator operator++(int)
        {
          my_iterator<map_iterator, vec_iterator> ret(*this);
          next();
          return ret;
        }

        const value_type *operator->()
        {
          return &operator*();
        }

        ~my_iterator()
        {
          clear_cache();
        }

        friend class ArgumentParser;
      };
    public:
      typedef my_iterator<madara::knowledge::KnowledgeMap::iterator,
            madara::knowledge::KnowledgeVector::iterator> iterator;

      typedef my_iterator<madara::knowledge::KnowledgeMap::const_iterator,
            madara::knowledge::KnowledgeVector::const_iterator> const_iterator;

      const_iterator begin() const
      {
        return const_iterator(*this);
      }

      static const_iterator end()
      {
        return const_iterator();
      }

    private:
      std::string prefix_;
      madara::knowledge::KnowledgeMap kmap_;
      const madara::knowledge::KnowledgeVector *kvec_;
    };
  }
}

#endif
