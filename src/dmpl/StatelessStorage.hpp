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

#ifndef _MADARA_STATELESS_HPP
#define _MADARA_STATELESS_HPP

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <climits>
#include <cassert>
#include <sstream>
#include <typeinfo>
#include <exception>
#include <madara/knowledge_engine/Thread_Safe_Context.h>
#include <madara/knowledge_engine/Thread_Safe_Context.h>
#include <madara/knowledge_engine/Knowledge_Update_Settings.h>
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

namespace Knowledge_Engine
{

namespace Containers
{

namespace StorageManager
{

using namespace ::Madara::Knowledge_Engine::Containers::__INTERNAL__;

namespace __INTERNAL__
{

template <unsigned int Size, unsigned int Dim>
class SizeManager
{
protected:
  static const unsigned int dim = Dim;
  static const unsigned int size = Size;
  SizeManager() {}
  SizeManager(unsigned int s) { resize(s); }
  void throw_error(unsigned int newSize)
  {
    std::ostringstream err;
    err << "Tried to resize dimension with static size " << size << " to size " << newSize << std::endl;
    throw std::range_error(err.str());
  }
public:
  unsigned int get_size() const { return size; }
  const unsigned int &get_size_ref() const { static unsigned int ref_size = Size; return ref_size; }
  unsigned int get_num_dims() const { return dim; }
  bool can_resize() const { return false; }
  void resize(unsigned int newSize) {
    if(newSize != 0 && newSize != size)
      throw_error(newSize);
  }
  bool check_bounds(unsigned int i) const
  {
    return i >= 0 && i < size;
  }
};

template <unsigned int Dim>
struct SizeManager<VAR_LEN, Dim>
{
protected:
  static const unsigned int dim = Dim;
  unsigned int size;
  SizeManager() : size(VAR_LEN) {}
  SizeManager(unsigned int s) : size(s > 0 ? s : VAR_LEN) {}
public:
  unsigned int get_size() const { return size; }
  const unsigned int &get_size_ref() const { return size; }
  unsigned int get_num_dims() const { return dim; }
  bool can_resize() const { return true; }
  void resize(unsigned int new_size) {
    if(new_size > 0)
      size = new_size;
  }
  bool check_bounds(unsigned int i) const
  {
    return i >= 0 && i < size;
  }
};

template <unsigned int Size, unsigned int Dim>
struct SizeManagerReference : protected SizeManager<Size, Dim>
{
public:
  SizeManager<Size, Dim> &get_size_manager()
  {
    return static_cast<SizeManager<Size, Dim>&>(*this);
  }
  const SizeManager<Size, Dim> &get_size_manager() const
  {
    return static_cast<const SizeManager<Size, Dim>&>(*this);
  }
  SizeManagerReference(SizeManager<Size, Dim> &sm) {}
};

template<unsigned int Dim>
struct SizeManagerReference<VAR_LEN, Dim>
{
protected:
  SizeManager<VAR_LEN, Dim> &size_manager;
public:
  SizeManager<VAR_LEN, Dim> &get_size_manager() { return size_manager; }
  const SizeManager<VAR_LEN, Dim> &get_size_manager() const { return size_manager; }
  SizeManagerReference(SizeManager<VAR_LEN, Dim> &sm) : size_manager(sm) {}
};

template <typename T>
struct Stateless
{
  typedef Stateless<T> this_type;
  typedef T data_type;

  template <typename X>
  class RefDimensionMixin;

  template <typename X>
  class RefBaseMixin;

  template<typename A, unsigned int Size, unsigned int Dims>
  class DimensionMixin : protected SizeManager<Size, Dims>
  {
  protected:
    typedef SizeManager<Size, Dims> size_mgr;

    size_mgr &get_size_mgr()
    {
      return static_cast<size_mgr &>(*this);
    }

    const size_mgr &get_size_mgr() const
    {
      return static_cast<const size_mgr &>(*this);
    }
  public:
    typedef A array_type;
    typedef T data_type;

    template <typename X>
    friend class ArrayReferenceReference;

    friend class identity<A>::type;

    template <typename X>
    friend class ArrayReference_0;

    template <typename X>
    friend class RefDimensionMixin;

    template <typename X>
    friend class RefBaseMixin;

    DimensionMixin(A &a, unsigned int i0 = 0)
      : size_mgr(i0) { }

    bool check_bounds(unsigned int i) const
    {
      return this->get_size_mgr().check_bounds(i);
    }

    unsigned int get_size() const
    {
      return this->get_size_mgr().get_size();
    }

    unsigned int get_num_dims() const
    {
      return this->get_size_mgr().get_num_dims();
    }

    bool can_resize() const
    {
      return this->get_size_mgr().can_resize();
    }

    void resize(unsigned int i)
    {
      return this->get_size_mgr().resize(i);
    }
  };

  template<typename A>
  class BaseMixin :
      protected SizeManager<1, 0>
  {
  protected:
    typedef SizeManager<1, 0> size_mgr;

    size_mgr &get_size_mgr()
    {
      return static_cast<size_mgr &>(*this);
    }

    const size_mgr &get_size_mgr() const
    {
      return static_cast<const size_mgr &>(*this);
    }

  public:
    template <typename X>
    friend class ArrayReferenceReference;

    friend class identity<A>::type;

    template <typename X>
    friend class ArrayReference_0;

    template <typename X>
    friend class RefDimensionMixin;

    template <typename X>
    friend class RefBaseMixin;

    BaseMixin(A &a)
    {
    }

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

  template<typename A>
  class RefDimensionMixin
    : public SizeManagerReference<A::static_size, A::dims>
  {
  protected:
    typedef SizeManagerReference<A::static_size, A::dims> size_mgr_ref;
    typedef SizeManager<A::static_size, A::dims> size_mgr;

    size_mgr &get_size_mgr()
    {
      return static_cast<size_mgr_ref &>(*this).get_size_manager();
    }

    const size_mgr &get_size_mgr() const
    {
      return static_cast<const size_mgr_ref &>(*this).get_size_manager();
    }
  public:
    typedef A array_type;
    typedef typename array_type::reference_type reference_type;
    typedef typename array_type::value_type value_type;

    template <typename X>
    friend class ArrayReferenceReference;

    friend class identity<A>::type;

    //template <typename X>
    //friend class ArrayReference_0;

    reference_type &get_reference()
    {
      return static_cast<reference_type&>(*this);
    }

    const reference_type &get_reference() const
    {
      return static_cast<reference_type&>(*this);
    }

    void append_index(unsigned int i)
    {
      //std::cerr << "dim append_index " << i << std::endl;
      this->get_reference().get_sub_type().append_index(i);
    }

    bool check_bounds(unsigned int i) const
    {
      return this->get_size_mgr().check_bounds(i);
    }

    unsigned int get_size() const
    {
      return this->get_size_mgr().get_size();
    }

    unsigned int get_num_dims() const
    {
      return this->get_size_mgr().get_num_dims();
    }

    bool can_resize() const
    {
      return this->get_size_mgr().can_resize();
    }

    RefDimensionMixin(A& a) : size_mgr_ref(a.template get_storage_mixin<0>().template get_size_mgr()) {}
  };

  template<typename A>
  class RefBaseMixin
    : public BaseReference<T, RefBaseMixin<A> >,
      public SizeManager<1, 0>
  {
  protected:
    typedef BaseReference<T, RefBaseMixin<A> > container_type;
    typedef SizeManager<1, 0> size_mgr;

    size_mgr &get_size_mgr()
    {
      return static_cast<size_mgr &>(*this);
    }

    const size_mgr &get_size_mgr() const
    {
      return static_cast<const size_mgr &>(*this);
    }
  public:
    typedef typename ArrayReference_0<typename A::storage_specifier>::type base_array_type;
    typedef ArrayReferenceReference<base_array_type> element_reference_type;
#ifdef USE_RVAL_REF
    typedef element_reference_type &&element_rvalue_type;
#endif
  protected:
    element_reference_type &get_parent()
    {
      return static_cast<element_reference_type &>(*this);
    }

    const element_reference_type &get_parent() const
    {
      return static_cast<const element_reference_type &>(*this);
    }

    base_array_type &get_array() const
    {
      return get_parent().array;
    }
  public:
    

    using container_type::operator=;

    template <typename X>
    friend class ArrayReferenceReference;

    friend class identity<A>::type;

    friend class identity<typename ArrayReference_0<this_type>::type >::type;

  protected:
    element_reference_type dereference()
#ifdef USE_RVAL_REF
    &
#endif
    {
      return static_cast<element_reference_type &>(*this);
    }

#ifdef USE_RVAL_REF
    element_rvalue_type dereference() &&
    {
      return std::move(static_cast<element_reference_type &>(*this));
    }
#endif

  public:
    const T &operator=(const RefBaseMixin &o)
    {
      return set(o);
    }

    void mark_modified()
    {
      //std::cerr << "Ref mark_modified " << this->get_name() << std::endl;

      const Knowledge_Update_Settings &settings = this->get_settings_cref();
      //Knowledge_Record *rec = this->get_context().get_record(this->get_name(), settings);
      //std::cerr << "Value: " << rec->to_integer() << std::endl;
      //std::cerr << "Size: " << rec->size() << std::endl;
      Variable_Reference ref = this->get_context().get_ref(this->get_name(), settings);
      //std::cerr << "Value: " << this->get_context().get(ref, settings) << std::endl;
      //this->get_context().mark_modified(ref);
      this->get_context().set(ref, this->get_context().get(ref,settings));
    }

    Thread_Safe_Context &get_context() const
    {
      return get_array().get_context();
    }

    /// Returns previous settings
    /*
    Knowledge_Update_Settings *set_settings(Knowledge_Update_Settings *new_settings)
    {
      return get_array().set_settings(new_settings);
      Knowledge_Update_Settings *old_settings = settings;
      settings = new_settings;
      return old_settings;
    }*/

    Knowledge_Update_Settings *get_settings() const
    {
      return get_array().get_settings();
    }

    const Knowledge_Update_Settings &get_settings_cref() const
    {
      return get_array().get_settings_cref();
    }

    Knowledge_Record get_knowledge_record() const {
      return this->get_context().get(this->get_name(), this->get_settings_cref());
    }

    T get() const
    {
      return knowledge_cast<T>(get_knowledge_record());
    }

    const Knowledge_Record &set_knowledge_record(const Knowledge_Record &in, const Knowledge_Update_Settings &settings)
    {
      this->get_context().set(this->get_name(), in, settings);
      return in;
    }

    const T &set(const T& in, const Knowledge_Update_Settings &settings)
    {
      set_knowledge_record(knowledge_cast(in), settings);
      return in;
    }

    using container_type::set;
    using container_type::set_knowledge_record;

#ifdef USE_RVAL_REF
    std::unique_ptr<std::ostringstream> name_str;
#else
    std::auto_ptr<std::ostringstream> name_str;
#endif

    RefBaseMixin(const A &a)
      : size_mgr(a.template get_storage_mixin<0>().get_size_mgr()),
        name_str(new std::ostringstream())
    {
      *name_str << a.get_name();
    }

    RefBaseMixin(const RefBaseMixin<A> &o)
      : size_mgr(o.get_size_mgr()),
        name_str(new std::ostringstream())
    {
      *name_str << o.name_str->str();
    }

#ifdef USE_RVAL_REF
    RefBaseMixin(RefBaseMixin<A> &&o)
      : size_mgr(o.get_size_mgr()),
        name_str(std::move(o.name_str)) { }
#endif

    std::string get_name() const
    {
      return name_str->str();
    }

    void append_index(unsigned int i)
    {
      //std::cerr << "base append_index " << i << std::endl;
      *name_str << "." << i;
    }
  };
};

} // End __INTERNAL__

template <typename T>
struct get_sm_info<__INTERNAL__::Stateless<T> >
{
  typedef __INTERNAL__::Stateless<T> sm_type;
  typedef T data_type;
};

}

}
}
}
#endif
