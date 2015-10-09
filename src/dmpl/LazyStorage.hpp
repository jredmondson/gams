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

#ifndef _MADARA_LAZY_HPP
#define _MADARA_LAZY_HPP

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <climits>
#include <cassert>
#include <sstream>
#include <typeinfo>
#include <exception>
#include <madara/knowledge/ThreadSafeContext.h>
#include <madara/knowledge/ThreadSafeContext.h>
#include <madara/knowledge/KnowledgeUpdateSettings.h>
#include "knowledge_cast.hpp"
#include "Reference.hpp"
#include "StorageManager.hpp"

#if __cplusplus >= 201103L
#include <array>

#define USE_RVAL_REF
#define USE_VAR_TMPL
#define USE_STD_ARRAY
#define USE_UNIQUE_PTR
#define USE_EMPLACE
#define USE_USING_TYPE
#define USE_STATIC_ASSERT
#endif

namespace Madara
{

namespace KnowledgeEngine
{

namespace Containers
{

namespace StorageManager
{

using namespace ::madara::knowledge::containers::__INTERNAL__;

template <typename T, typename S>
struct Lazy
{
  typedef Lazy<T, S> this_type;
  typedef T data_type;
  typedef S storage_type;

  template<typename A>
  class BaseMixin
  {
  public:
#ifdef USE_UNIQUE_PTR
    typedef std::unique_ptr<S> ptr_type;
#else
    typedef S *ptr_type;
#endif
    typedef std::vector<ptr_type> vector_type;
    typedef std::vector<unsigned int> dim_sizes_t;
    dim_sizes_t dim_sizes;
    vector_type stored_data;

    template <typename X>
    friend class ArrayReferenceReference;

    //template <typename X>
    //friend class identity<typename ArrayReference_0<X>::type >::type;

    /** Just clears existing storage.
     * LazyStorage never preallocates space (to permit unbounded VAR_LEN initial dimension)
     */
    void init_storage()
    {
      stored_data.clear();
    }

    BaseMixin(A &a) {}

    BaseMixin(const BaseMixin &o) : dim_sizes(o.dim_sizes)
    {
      for(typename vector_type::iterator it = stored_data.begin(); it != stored_data.end(); ++it)
      {
        if(*it)
        {
#ifdef USE_UNIQUE_PTR
          ptr_type ptr = make_unique<S>(**it);
#else
          ptr_type ptr = new S(**it);
#endif
          stored_data.push_back(ptr);
        }
        else
        {
#ifdef USE_UNIQUE_PTR
          stored_data.push_back(std::unique_ptr<S>());
#else
          stored_data.push_back(NULL);
#endif
        }
      }
    }

    ~BaseMixin()
    {
#ifndef USE_UNIQUE_PTR
      for(typename vector_type::iterator it = stored_data.begin(); it != stored_data.end(); ++it)
      {
        delete *it;
      }
#endif
    }

#ifdef USE_RVAL_REF
    BaseMixin(BaseMixin &&o)
      : dim_sizes(std::move(o.dim_sizes)), stored_data(std::move(o.stored_data)) {}
#endif

    unsigned int get_size() const
    {
      return 1;
    }

    unsigned int get_num_dims() const
    {
      return 0;
    }

    bool can_resize() const
    {
      return false;
    }
  };

  template<typename A, unsigned int Size, unsigned int Dims>
  class DimensionMixin
  {
  public:
    typedef A array_type;

    static const unsigned int static_size = Size;
    static const unsigned int num_dims = Dims;

    typedef typename ArrayReference_0<this_type>::type base_type;
    typedef BaseMixin<base_type> base_mixin;

    template <typename X>
    friend class ArrayReferenceReference;

    //friend class identity<A>::type;

    //template <typename X>
    //friend class identity<typename ArrayReference_0<X>::type >::type;

    base_mixin &get_base_mixin()
    {
      return static_cast<base_mixin &>(static_cast<base_type &>(static_cast<A &>(*this)));
    }

    const base_mixin &get_base_mixin() const
    {
      return static_cast<const base_mixin &>(static_cast<const base_type &>(static_cast<const A &>(*this)));
    }

  private:
    void size_sanity(unsigned int newSize)
    {
      if(static_size != VAR_LEN && (newSize != 0 && newSize != static_size))
      {
        std::ostringstream err;
        err << "Tried to resize dimension with static size " << static_size << " to size " << newSize << std::endl;
        throw std::range_error(err.str());
      }
    }

  public:
    void resize(unsigned int i)
    {
      if(i != get_size())
      {
        size_sanity(i);
        get_base_mixin().dim_sizes[num_dims - 1] = i;
        get_base_mixin().init_storage();
      }
    }

    unsigned int get_size() const
    {
      return static_size == VAR_LEN ? get_base_mixin().dim_sizes[num_dims - 1] : static_size;
    }

    bool check_bounds(unsigned int i) const
    {
      return i >= 0 && i < get_size();
    }

    unsigned int get_num_dims() const
    {
      return num_dims;
    }

    bool can_resize() const
    {
      return static_size == VAR_LEN;
    }

  public:
    DimensionMixin(A &a, unsigned int i0 = 0)
    {
      size_sanity(i0);
      unsigned int s = (i0 == 0 ? static_size : i0);
      a.dim_sizes.push_back(s);
    }
  };

  template<typename A>
  class RefDimensionMixin
  {
  public:
    typedef A array_type;
    static const unsigned int static_size = A::static_size;
    static const unsigned int num_dims = A::dims;
    typedef typename array_type::base_type array_base_type;
    typedef typename array_type::reference_type reference_type;
    typedef typename array_type::base_reference_type base_reference_type;
    typedef typename array_type::value_type value_type;
    typedef BaseMixin<array_base_type> base_mixin;

    template <typename X>
    friend class ArrayReferenceReference;

    //friend class identity<A>::type;

    //template <typename X>
    //friend class identity<typename ArrayReference_0<X>::type >::type;

    reference_type &get_reference()
    {
      return static_cast<reference_type&>(*this);
    }

    const reference_type &get_reference() const
    {
      return static_cast<const reference_type&>(*this);
    }

    base_reference_type &get_base_reference()
    {
      return static_cast<base_reference_type&>(get_reference());
    }

    const base_reference_type &get_base_reference() const
    {
      return static_cast<const base_reference_type&>(get_reference());
    }

    base_mixin &get_base_mixin()
    {
      return get_base_reference().base_mixin;
    }

    const base_mixin &get_base_mixin() const
    {
      return get_base_reference().base_mixin;
    }

    unsigned int get_size() const
    {
      return static_size == VAR_LEN ? get_base_mixin().dim_sizes[num_dims - 1] : static_size;
    }

    void append_index(unsigned int i)
    {
      //std::cerr << "append_index: " << this->get_reference().offset << " += " <<
      //  i << " * " << this->get_reference().get_multiplier() << 
      //  "  num_dims: " << this->get_reference().get_size_mgr().get_num_dims() << std::endl;
      
      unsigned int mult = this->get_reference().get_multiplier();
      if(mult == VAR_LEN)
        throw std::range_error("Lazy storage manager does not support unbounded VAR_LEN dimensions.");
      this->get_reference().offset += i * mult;
    }

    unsigned int get_num_dims() const
    {
      return num_dims;
    }

    bool can_resize() const
    {
      return static_size == VAR_LEN;
    }

    bool check_bounds(unsigned int i) const
    {
      return i >= 0 && i < get_size();
    }

    RefDimensionMixin(A& a) {}
  };

  template<typename A>
  class RefBaseMixin
  {
  public:
    typedef storage_type  element_reference_type;
    typedef storage_type  element_rvalue_type;

    typedef typename ArrayReference_0<typename A::storage_specifier>::type base_array_type;
    typedef ArrayReferenceReference<base_array_type> base_ref_type;

    typedef typename BaseMixin<A>::ptr_type ptr_type;

    const std::string base_name;
    BaseMixin<A> &base_mixin;
    unsigned int offset;

    template <typename X>
    friend class ArrayReferenceReference;

    //friend class identity<A>::type;

    //template <typename X>
    //friend class identity<typename ArrayReference_0<X>::type >::type;

    RefBaseMixin(A &a)
      : base_name(a.get_name()), base_mixin(static_cast<BaseMixin<A> &>(a)), offset(0)
    { }

    RefBaseMixin(const RefBaseMixin<A> &o)
      : base_name(o.base_name), base_mixin(o.base_mixin), offset(o.offset)
    { }

#ifdef USE_RVAL_REF
    RefBaseMixin(RefBaseMixin<A> &&o)
      : base_name(std::move(o.base_name)), base_mixin(o.base_mixin), offset(o.offset) { }
#endif

  protected:
    base_ref_type &get_base_ref()
    {
      return static_cast<base_ref_type &>(*this);
    }

    base_array_type &get_base_array()
    {
      return static_cast<base_array_type &>(base_mixin);
    }

    element_reference_type dereference()
    {
      if(offset >= base_mixin.stored_data.size())
        base_mixin.stored_data.resize(offset + 1);
      ptr_type &data_ptr = base_mixin.stored_data[offset];
      if(data_ptr == NULL)
      {
#ifdef USE_UNIQUE_PTR
          data_ptr = make_unique<storage_type>(get_base_array().get_context(), get_base_array().get_settings(), get_name());
#else
          data_ptr = new storage_type(get_base_array().get_context(), get_base_array().get_settings(), get_name());
#endif
      }
      return *data_ptr;
    }

  public:
    std::string get_name() const
    {
      std::ostringstream ret;
      ret << base_name;
      unsigned int o = offset;
      for(typename BaseMixin<A>::dim_sizes_t::const_reverse_iterator it = base_mixin.dim_sizes.rbegin(); it != base_mixin.dim_sizes.rend(); ++it)
      {
        unsigned int mult = 1;
        typename BaseMixin<A>::dim_sizes_t::const_reverse_iterator it2 = it;
        ++it2;
        for(; it2 != base_mixin.dim_sizes.rend(); ++it2)
        {
          if(*it == VAR_LEN)
            throw std::runtime_error("Lazy storage manager does not support unbound VAR_LEN dimensions, other than the first dimension.");
          mult *= *it2;
        }
        ret << "." << o / mult;
        o %= mult;
      }
      return ret.str();
    }

    void append_index(unsigned int i)
    {
    }
  };
};

#if 0
template <typename T, typename S>
struct Proactive
{
  typedef T data_type;
  typedef S storage_type;

  template<typename A>
  class DimensionMixin
  {
  };

  template<typename A>
  class BaseMixin
  {
  public:
    typedef T data_type;
    typedef S storage_type;
    typedef storage_type getter_type;
    typedef std::vector<T> vector_type;
    vector_type stored_data;
  };

  template<typename A>
  class RefDimensionMixin
  {
  public:
  };

  template<typename A>
  class RefBaseMixin
  {
  public:
  };
};
#endif

template <typename T, typename S>
struct get_sm_info<Lazy<T,S> >
{
  typedef Lazy<T,S> sm_type;
  typedef T data_type;
  typedef S storage_type;
};

#if 0
template <typename T, typename S>
struct get_sm_info<Proactive<T,S> >
{
  typedef Proactive<T,S> sm_type;
  typedef T data_type;
  typedef S storage_type;
};
#endif

}

}
}
}
#endif
