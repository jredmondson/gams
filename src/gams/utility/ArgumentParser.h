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

#ifndef GAMS_UTILITY_ARGUMENT_PARSER_H_
#define GAMS_UTILITY_ARGUMENT_PARSER_H_

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
    /**
     * Compare object suitable for use with standard library.
     * Compares based on radix order: shorter strings always come before
     * longer strings, and equal length strings are sorted in lexicographicaly
     * order). Note that any strings containing only an integer of some length
     * (with no leading zeros) will be sorted by numerical value implictly
     **/
    class RadixLess
    {
    public:
      bool operator()(const std::string &lhs, const std::string &rhs) const
      {
        size_t lsz = lhs.size(), rsz = rhs.size();
        if(lsz < rsz)
          return true;
        else if(rsz < lsz)
          return false;
        else
          return std::less<std::string>()(lhs, rhs);
      }
    };

    /**
     * Converts a knowledge map into a knowledge vector by adding the values
     * from the map in numerical order (as defined by radix order: shorter
     * strings always come before longer strings, and equal length strings
     * are sorted in lexicographicaly order)
     *
     * keys which don't start with a digit are ignored
     **/
    inline madara::knowledge::KnowledgeVector kmap2kvec(
      const madara::knowledge::KnowledgeMap &kmap)
    {
      using namespace madara::knowledge;
      std::map<std::string, KnowledgeRecord,
               RadixLess> map(kmap.begin(), kmap.end());
      KnowledgeVector ret;
      for(KnowledgeMap::const_iterator i = map.begin(); i != map.end(); ++i)
      {
        char c = i->first[0];
        if(c >= '0' && c <= '9')
          ret.push_back(i->second);
      }
      return ret;
    }

    class ArgumentParser
    {
    public:
      ArgumentParser() : kmap_() {}

      ArgumentParser(const madara::knowledge::KnowledgeMap &kmap)
        : kmap_(&kmap) {}

    private:
      template<class map_iterator>
      class my_iterator : public std::iterator<
        std::input_iterator_tag, typename map_iterator::value_type >
      {
      public:
        typedef typename my_iterator::value_type value_type;
      private:
        my_iterator() : dummy_(0), parent_(nullptr), cache_valid_(0) {}

        my_iterator(const ArgumentParser &parent)
          : dummy_(0), parent_(&parent), it_k_(parent_->kmap_->begin()),
            it_v_(it_k_), cache_valid_(0)
        {
          detect_old_style();
        }

      public:
        my_iterator(const my_iterator &o)
          : dummy_(0), parent_(o.parent_), it_k_(o.it_k_), it_v_(it_v_),
            cache_valid_(0) {}

        my_iterator &operator=(const my_iterator &o)
        {
          parent_ = o.parent_;
          it_k_ = o.it_k_;
          it_v_ = o.it_v_;
          clear_cache();
          return *this;
        }

        ~my_iterator() { clear_cache(); }

      private:
        union {
#ifdef CPP11
          mutable int dummy_; // initialize this by default
          mutable value_type cache_;
#else
          mutable char cache_[sizeof(value_type)];
          // Try to ensure that any possible alignment requirements are met:
          mutable uint64_t dummy_;
          mutable long double dummy2_;
          mutable void * dummy3_;
#endif
        };

#ifdef CPP11
        value_type *cache() const
        {
          return cache_valid_ ? &cache_ : nullptr;
        }
#else
        value_type *cache() const
        {
          return cache_valid_ ?
                     reinterpret_cast<value_type*>(&cache_) :
                     nullptr;
        }
#endif

        const ArgumentParser *parent_;
        map_iterator it_k_;
        map_iterator it_v_;

        mutable bool cache_valid_;
      public:
        bool at_end() const
        {
          return parent_ == nullptr || parent_->kmap_ == nullptr ||
                 it_k_ == parent_->kmap_->end();
        }

        bool operator==(const my_iterator &o) const
        {
          bool ae1 = at_end(), ae2 = o.at_end();
          if(ae1 || ae2)
            return ae1 && ae2;
          else
            return parent_ == o.parent_ &&
                   it_k_ == o.it_k_;
        }

        bool operator!=(const my_iterator &o) const
        {
          return !(*this == o);
        }

      public:
        std::string name() const
        {
          return (!is_old_style()) ?
                      it_k_->first :
                      it_k_->second.to_string();
        }

        typename value_type::second_type value() const
        {
          return it_v_->second;
        }

        bool is_old_style() const
        {
          return it_k_ != it_v_;
        }

      private:
        bool name_is_old_style() const
        {
          char c = it_k_->first[0];
          if(c >= '0' && c <= '9')
            return true;
          return false;
        }

        void detect_old_style()
        {
          if(at_end())
            return;
          if(!at_end() && name_is_old_style() && !is_old_style())
            ++it_v_;
          if(it_v_ == parent_->kmap_->end())
            it_k_ = it_v_;
        }

        void clear_cache()
        {
          if(cache())
          {
            cache()->~value_type();
            cache_valid_ = false;
          }
        }

      public:
        void next()
        {
          ++it_v_;
          it_k_ = it_v_;
          detect_old_style();
          clear_cache();
        }

        const value_type &operator*() const
        {
          if(cache())
            return *cache();
          value_type *ret = new((void*)&cache_) value_type(name(),value());
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
          my_iterator ret(*this);
          next();
          return ret;
        }

        const value_type *operator->()
        {
          return &operator*();
        }

        friend class ArgumentParser;
      };
    public:
      typedef my_iterator<madara::knowledge::KnowledgeMap::iterator> iterator;

      typedef my_iterator<madara::knowledge::KnowledgeMap::const_iterator>
        const_iterator;

      const_iterator begin() const
      {
        return const_iterator(*this);
      }

      static const_iterator end()
      {
        return const_iterator();
      }

    private:
      const madara::knowledge::KnowledgeMap *kmap_;
    };
  }
}

#endif
