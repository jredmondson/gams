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

#ifndef _MADARA_ARRAY_HPP
#define _MADARA_ARRAY_HPP

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

#if __cplusplus >= 201103L
#include <array>

#define USE_RVAL_REF
#define USE_VAR_TMPL
#define USE_STD_ARRAY
#define USE_UNIQUE_PTR
#define USE_EMPLACE
#define USE_USING_TYPE
#define USE_STATIC_ASSERT
#else
#define nullptr NULL
#endif



namespace Madara
{

#ifdef USE_UNIQUE_PTR
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args )
{
  return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}
#endif

namespace Knowledge_Engine
{
namespace Containers
{

static const unsigned int VAR_LEN = UINT_MAX;

namespace StorageManager
{

namespace __INTERNAL__
{

template<typename T>
class Stateless;

}

template<typename T>
struct get_sm_info;

}

#ifdef USE_VAR_TMPL
template <typename T, unsigned int d0 = 0, unsigned int ...dN>
class ArrayReference;
#else

template <typename T, unsigned int d0, unsigned int d1 = 0, unsigned int d2 = 0, unsigned int d3 = 0,
                      unsigned int d4 = 0, unsigned int d5 = 0, unsigned int d6 = 0, unsigned int d7 = 0,
                      unsigned int d8 = 0, unsigned int d9 = 0>
class ArrayReference;
#endif

namespace __INTERNAL__
{

template<typename T>
struct ArrayReference_0
{
#ifdef USE_VAR_TMPL
  typedef class ArrayReference<T> type;
#else
  typedef ArrayReference<T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0> type;
#endif
};

class ArrayReferenceBase : public ContextHost
{
public:
  const static unsigned int dims = 0;

  template<typename T>
  friend class ArrayReferenceReference;
protected:
  //typedef ArrayReferenceBase forwarded_type;

protected:
  const static unsigned int static_size = 1;
  const static unsigned int dim0 = 0;
  typedef ArrayReferenceBase subarray_type;
  typedef ArrayReferenceBase sub0;

  std::string name;

  ArrayReferenceBase(Thread_Safe_Context &con,
      Knowledge_Update_Settings *settings = nullptr,
      const std::string &varName = "") :
    ContextHost(con, settings), name(varName) {}
public:
  const std::string &get_name() const { return name; }

protected:
  void get_dims(std::vector<int> &out) const {}
};

template<typename T>
class ArrayReferenceReference
  : protected ArrayReferenceReference<typename T::subarray_type>,
    public T::sm_type::template RefDimensionMixin<T>
{
protected:
  typedef T array_type;
  typedef typename T::subarray_type subarray_type;
  typedef typename array_type::sm_info sm_info;
  typedef typename array_type::sm_type sm_type;
  typedef typename sm_type::template RefDimensionMixin<array_type> storage_mixin;

  /*
  size_mgr &get_size_mgr()
  {
    return static_cast<size_mgr &>(*this).get_size_manager();
  }

  const size_mgr &get_size_mgr() const
  {
    return static_cast<const size_mgr &>(*this)._size_manager();
  }
  */

  storage_mixin &get_storage_mixin()
  {
    return static_cast<storage_mixin &>(*this);
  }

  const storage_mixin &get_storage_mixin() const
  {
    return static_cast<const storage_mixin &>(*this);
  }
public:
  typedef typename array_type::value_type value_type;

  typedef ArrayReferenceReference<array_type> index_as_type;
  typedef ArrayReferenceReference<typename array_type::subarray_type> sub_type;
  typedef typename sub_type::index_as_type indexed_type;
#ifdef USE_RVAL_REF
  typedef ArrayReferenceReference<array_type> &&rvalue_index_as_type;
  typedef typename sub_type::rvalue_index_as_type rvalue_indexed_type;
#endif

protected:
  array_type &get_array()
  {
    return static_cast<array_type &>(sub_type::get_array());
  }

public:

  friend class ArrayReferenceBase;
  friend class identity<array_type>::type;
  friend class identity<sm_type>::type;

  template<typename A>
  friend class sm_type::BaseMixin;

  template<typename A>
  friend class sm_type::RefBaseMixin;

  template<typename A, unsigned int Size, unsigned int Dims>
  friend class sm_type::DimensionMixin;

  template<typename A>
  friend class sm_type::RefDimensionMixin;

  template<typename A>
  friend class ArrayReferenceReference;

#ifdef USE_VAR_TMPL
  template<typename X, unsigned int x0, unsigned int... xN>
  friend class ArrayReference;
#else
  template <typename X, unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3,
                        unsigned int x4, unsigned int x5, unsigned int x6, unsigned int x7,
                        unsigned int x8, unsigned int x9>
  friend class ArrayReference;
#endif

  sub_type &get_sub_type()
  {
    return static_cast<sub_type &>(*this);
  }

  const sub_type &get_sub_type() const
  {
    return static_cast<const sub_type &>(*this);
  }

  unsigned int get_multiplier() const
  {
    //std::cerr << "get_multiplier  dims == " << dims << "  size == " << get_size<0>() << std::endl;
    const unsigned int num_dims = get_num_dims();
    if(num_dims <= 1)
      return 1;
    unsigned int s = get_sub_type().get_size();
    if(num_dims == 2)
      return s;
    if(s == VAR_LEN)
      return VAR_LEN;
    return s * get_sub_type().get_multiplier();
  }

  void mark_modified()
  {
    //std::cerr << "Reference mark_modified " << this->get_name() << std::endl;
    get_array().mark_modified(*this);
  }

  /**
   * C++11 support is highly recommended for multi-dimensional arrays;
   * without it, each index operation creates pointless copies of ArrayReferenceReference
   * object
   */
#ifdef USE_RVAL_REF
  indexed_type operator[](unsigned int i) &;

  rvalue_indexed_type operator[](unsigned int i) &&;
#else
  indexed_type operator[](unsigned int i);
#endif

  operator array_type() const
  {
    array_type ret(this->get_context(), this->get_settings(), this->get_name());
    //std::cerr << "cast to array" << std::endl;
    return ret;
  }

protected:
  ArrayReferenceReference(array_type &v)
    : ArrayReferenceReference<typename array_type::subarray_type>(v),
      storage_mixin(v) {}

  index_as_type dereference()
#ifdef USE_RVAL_REF
  &
#endif
  {
    return *this;
  }

#ifdef USE_RVAL_REF
  rvalue_index_as_type dereference() &&
  {
    return std::move(*this);
  }
#endif

public:
  void append_index(unsigned int i)
  {
    get_storage_mixin().append_index(i);
  }

  bool check_bounds(unsigned int i) const
  {
    return get_storage_mixin().check_bounds(i);
  }

  unsigned int get_size() const
  {
    return get_storage_mixin().get_size();
  }

  unsigned int get_num_dims() const
  {
    return get_storage_mixin().get_num_dims();
  }

  bool can_resize() const
  {
    return get_storage_mixin().can_resize();
  }
};

template<typename R>
#ifdef USE_VAR_TMPL
class ArrayReferenceReference<ArrayReference<R>>
#else
class ArrayReferenceReference<ArrayReference<R, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0> >
#endif
  : public ArrayReference_0<R>::type::sm_type::template RefBaseMixin<typename ArrayReference_0<R>::type>
{
public:
  typedef typename ArrayReference_0<R>::type array_type;
protected:
  array_type &array;

  typedef R storage_specifier;
  typedef StorageManager::get_sm_info<R> sm_info;
  typedef typename sm_info::sm_type sm_type;
  typedef typename sm_info::data_type value_type;
  typedef typename sm_type::template RefBaseMixin<array_type> storage_mixin;

  storage_mixin &get_storage_mixin()
  {
    return static_cast<storage_mixin &>(*this);
  }

  const storage_mixin &get_storage_mixin() const
  {
    return static_cast<const storage_mixin &>(*this);
  }

  array_type &get_array() const
  {
    return array;
  }
public:
  using storage_mixin::operator=;

  const value_type &operator=(const ArrayReferenceReference &o)
  {
    return storage_mixin::operator=(o);
  }

  ArrayReferenceReference(array_type &v)
    : storage_mixin(v), array(v)
  { }

  ArrayReferenceReference(const ArrayReferenceReference<array_type> &o)
    : storage_mixin(o.get_storage_mixin()), array(o.array)
  {
    //std::cerr << "Copying: " << o.name_str->str() << " to " << name_str->str() << std::endl;
  }

#ifdef USE_RVAL_REF
  ArrayReferenceReference(ArrayReferenceReference &&o)
    : storage_mixin(std::move(o.get_storage_mixin())), array(o.array)
  {
    //std::cerr << "ArrayReferenceReference move ctor called" << std::endl;
  }
#endif

  typedef typename storage_mixin::element_reference_type index_as_type;
#ifdef USE_RVAL_REF
  typedef typename storage_mixin::element_rvalue_type rvalue_index_as_type;
#endif

  std::string get_name() const
  {
    return get_storage_mixin().get_name();
  }

  unsigned int get_multiplier() const
  {
    return 1;
  }

  bool check_bounds(unsigned int i) const
  {
    throw std::runtime_error("check_bounds called of element reference");
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

  void append_index(unsigned int i)
  {
    get_storage_mixin().append_index(i);
  }

  friend class ArrayReferenceBase;
  
  template<typename A>
  friend class ArrayReferenceReference;

  friend class identity<sm_type>::type;

  template<typename A>
  friend class sm_type::BaseMixin;

  template<typename A>
  friend class sm_type::RefBaseMixin;

  template<typename A, unsigned int Size, unsigned int Dims>
  friend class sm_type::DimensionMixin;

  template<typename A>
  friend class sm_type::RefDimensionMixin;

#ifdef USE_VAR_TMPL
  template<typename X, unsigned int x0, unsigned int... xN>
  friend class ArrayReference;
#else
  template <typename X, unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3,
                        unsigned int x4, unsigned int x5, unsigned int x6, unsigned int x7,
                        unsigned int x8, unsigned int x9>
  friend class ArrayReference;
#endif
};


void throw_range_error(std::string name, int i, int max)
{
  std::ostringstream err;
  err << "Index " << i << " of " << name << " out of bounds (required: 0 <= index < " << max << ")";
  throw std::range_error(err.str());
}

template<typename T>
inline typename ArrayReferenceReference<T>::indexed_type ArrayReferenceReference<T>::operator[](unsigned int i)
#ifdef USE_RVAL_REF
 &
#endif
{
  //std::cerr << "indexing dim " << get_size_mgr().get_num_dims() << " size " << get_size_mgr().get_size() << " i: " << i << std::endl;;
  if(!this->check_bounds(i))
    throw_range_error(this->get_name(), i, this->get_size());
  //std::cerr << "index op lvalue: " << this->get_name() << std::endl;
  ArrayReferenceReference<T> ret(*this);
  ret.append_index(i);
  //std::cerr << "index lvalue: " << ret.get_name() << std::endl;
  return static_cast<sub_type&>(ret).dereference();
}

#ifdef USE_RVAL_REF
template<typename T>
inline typename ArrayReferenceReference<T>::rvalue_indexed_type ArrayReferenceReference<T>::operator[](unsigned int i) &&
{
  //std::cerr << "indexing dim " << get_size_mgr().get_num_dims() << " size " << get_size_mgr().get_size() << " i: " << i << std::endl;;
  if(!this->check_bounds(i))
    throw_range_error(this->get_name(), i, this->get_size());
  //std::cerr << "index op rvalue: " << this->get_name() << std::endl;
  this->append_index(i);
  return std::move(static_cast<sub_type&>(*this)).dereference();
}
#endif

} // end __INTERNAL__


#ifdef USE_VAR_TMPL
template <typename T, unsigned int d0, unsigned int ...dN>
class ArrayReference
  : protected ArrayReference<T, dN...>,
    public StorageManager::get_sm_info<T>::sm_type::template
        DimensionMixin<ArrayReference<T, d0, dN...>, d0, ArrayReference<T, dN...>::dims + 1 >
#else
template <typename T, unsigned int d0, unsigned int d1, unsigned int d2, unsigned int d3,
                      unsigned int d4, unsigned int d5, unsigned int d6, unsigned int d7,
                      unsigned int d8, unsigned int d9>
class ArrayReference
  : protected ArrayReference<T, d1, d2, d3, d4, d5, d6, d7, d8, d9, 0>,
    public StorageManager::get_sm_info<T>::sm_type::template
        DimensionMixin<ArrayReference<T, d0, d1, d2, d3, d4, d5, d6, d7, d8, d9>, d0,
          ArrayReference<T, d1, d2, d3, d4, d5, d6, d7, d8, d9, 0>::dims + 1 >
#endif
{
protected:
#ifdef USE_VAR_TMPL
  typedef ArrayReference<T, dN...> raw_subarray_type;
#else
  typedef ArrayReference<T, d1, d2, d3, d4, d5, d6, d7, d8, d9, 0> raw_subarray_type;
#endif
public:
  typedef T storage_specifier;
  typedef StorageManager::get_sm_info<T> sm_info;
  typedef typename sm_info::sm_type sm_type;
  typedef typename sm_info::data_type value_type;
  typedef raw_subarray_type subarray_type;
  //typedef typename raw_subarray_type::forwarded_type subarray_type;

#ifdef USE_VAR_TMPL
  typedef ArrayReference<T, d0, dN...> this_type;
#else
  typedef ArrayReference<T, d0, d1, d2, d3, d4, d5, d6, d7, d8, d9> this_type;
#endif
  typedef typename __INTERNAL__::ArrayReference_0<storage_specifier>::type base_type;
  //typedef typename raw_subarray_type::forwarded_type subarray_type;
  typedef __INTERNAL__::ArrayReferenceReference<this_type> reference_type;
  typedef __INTERNAL__::ArrayReferenceReference<base_type> base_reference_type;
  typedef typename __INTERNAL__::ArrayReferenceReference<subarray_type>::index_as_type index_type;
public:
  const static unsigned int static_size = d0;
  const static unsigned int dims = raw_subarray_type::dims + 1;
  typedef typename sm_type::template DimensionMixin<this_type, d0, dims> storage_mixin;
  typedef typename sm_type::template BaseMixin<base_type> base_storage_mixin;

  template<unsigned int dimension, bool dummy = true>
  struct get_dimension_type
  {
    typedef typename raw_subarray_type::template get_dimension_type<dimension - 1, dummy>::type type;
  };

  template<bool dummy>
  struct get_dimension_type<0, dummy>
  {
    typedef this_type type;
  };

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  typename get_dimension_type<dimension>::type &get_dimension()
  {
    return static_cast<typename get_dimension_type<dimension>::type &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const typename get_dimension_type<dimension>::type &get_dimension() const
  {
    return static_cast<const typename get_dimension_type<dimension>::type &>(*this);
  }

protected:
  template<unsigned int dimension, bool dummy = true>
  struct get_next_dimension_type
  {
    typedef typename raw_subarray_type::template get_dimension_type<dimension - 1, dummy>::type type;
  };

  template<bool dummy>
  struct get_next_dimension_type<0, dummy>
  {
    typedef raw_subarray_type type;
  };

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const typename get_next_dimension_type<dimension>::type &get_next_dimension() const
  {
    return static_cast<const typename get_next_dimension_type<dimension>::type &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const typename get_dimension_type<dimension>::type::storage_mixin &get_storage_mixin() const
  {
    return static_cast<const typename get_dimension_type<dimension>::type::storage_mixin &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  typename get_dimension_type<dimension>::type::storage_mixin &get_storage_mixin()
  {
    return static_cast<typename get_dimension_type<dimension>::type::storage_mixin &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const typename get_dimension_type<dimension>::type::base_storage_mixin &get_base_storage_mixin() const
  {
    return static_cast<const typename get_dimension_type<dimension>::type::base_storage_mixin &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  typename get_dimension_type<dimension>::type::base_storage_mixin &base_get_storage_mixin()
  {
    return static_cast<typename get_dimension_type<dimension>::type::base_storage_mixin &>(*this);
  }

public:
  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  unsigned int get_size() const
  {
    return get_storage_mixin<dimension>().get_size();
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const unsigned int &get_size_ref() const
  {
    return get_storage_mixin<dimension>().get_size_ref();
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  void resize(unsigned int new_size)
  {
    get_storage_mixin<dimension>().resize(new_size);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  bool can_resize() const
  {
    return get_storage_mixin<dimension>().can_resize();
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  bool check_bounds(unsigned int i) const
  {
    return get_storage_mixin<dimension>().check_bounds(i);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  unsigned int get_multiplier() const
  {
    //std::cerr << "get_multiplier  dims == " << dims << "  size == " << get_size<0>() << std::endl;
    const unsigned int num_dims = get_dimension<dimension>().dims;
    if(num_dims <= 1)
      return 1;
    unsigned int s = get_next_dimension<dimension>().get_size<dimension>();
    if(num_dims == 2)
      return s;
    if(s == VAR_LEN)
      return VAR_LEN;
    return s * static_cast<const typename get_next_dimension_type<dimension>::type*>(this)->get_multiplier<dimension>();
  }

public:
#ifdef USE_USING_TYPE
  template<class E = value_type>
  using get_vector_type = std::vector<typename raw_subarray_type::template get_vector_type<E> >;
  typedef get_vector_type<> vector_type;
#endif
  template<class E = value_type>
  struct get_vector_type_compat
  {
    typedef std::vector<typename raw_subarray_type::template get_vector_type_compat<E>::type > type;
  };
#ifndef USE_USING_TYPE
  typedef typename get_vector_type_compat<>::type vector_type;
#endif

#if defined(USE_STD_ARRAY) && defined(USE_USING_TYPE)
  template<class E = value_type>
  using get_array_type = std::array<typename raw_subarray_type::template
    get_array_type<E>, static_size == VAR_LEN ? 1 : static_size>;

  typedef get_array_type<> array_type;
#else
  template<class E = value_type>
  struct get_array_type
  {
    typedef typename raw_subarray_type::template
      get_array_type<E>::type type[static_size == VAR_LEN ? 1 : static_size];
  };
  typedef typename get_array_type<>::type array_type;
#endif

private:
  void check_var_len(const char *fname)
  {
    if(static_size == VAR_LEN && get_size<0>() == VAR_LEN)
      throw std::range_error(std::string(fname) + "() called on ArrayReference with unbounded VAR_LEN dimension");
  }

public:
  void mark_modified()
  {
    //std::cerr << "Array mark_modified " << this->get_name() << std::endl;
    check_var_len("mark_modified");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::mark_modified((*this)[i]);
    }
  }

  template<class E>
  void get_into(E &out)
  {
    check_var_len("get_into");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::get_into(out[i], (*this)[i]);
    }
  }

  template<class E>
  void get_into(std::vector<E> &out)
  {
    check_var_len("get_into");
    out.clear();
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::get_into(out, (*this)[i]);
    }
  }

  template<class E>
  void set_from(const E &in)
  {
    check_var_len("set_from");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::set_from(in[i], (*this)[i]);
    }
  }

  template<class E>
  void update_from(const E &in)
  {
    check_var_len("update_from");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::update_from(in[i], (*this)[i]);
    }
  }

  template<class E>
  void push_all(E &in)
  {
    check_var_len("push_all");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::push_all(in[i]);
    }
  }

  template<class E>
  void pull_all(E &in)
  {
    check_var_len("pull_all");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::pull_all(in[i]);
    }
  }

  template<class E>
  void pull_all_keep_local(E &in)
  {
    check_var_len("pull_all_keep_local");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::pull_all_keep_local(in[i]);
    }
  }

protected:
  void mark_modified(reference_type ref)
  {
    //std::cerr << "Array sub mark_modified " << ref.get_name() << std::endl;
    check_var_len("mark_modified");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::mark_modified(ref[i]);
    }
  }

  template<class E>
  void get_into(E &out, reference_type ref)
  {
    check_var_len("get_into");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::get_into(out[i], ref[i]);
    }
  }

  template<class E>
  void get_into(std::vector<E> &out, reference_type ref)
  {
    check_var_len("get_into");
#ifdef USE_EMPLACE
    out.emplace_back();
#else
    out.push_back(E());
#endif
    E &self = out.back();
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::get_into(self, ref[i]);
    }
  }

  template<class E>
  void set_from(const E &in, reference_type ref)
  {
    check_var_len("set_from");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::set_from(in[i], ref[i]);
    }
  }

  template<class E>
  void update_from(const E &in, reference_type ref)
  {
    check_var_len("update_from");
    for(int i = 0; i < get_size<0>(); i++)
    {
      raw_subarray_type::update_from(in[i], ref[i]);
    }
  }

protected:
  typedef this_type forwarded_type;
public:

#ifdef USE_VAR_TMPL
  ArrayReference(Thread_Safe_Context &con, const std::string &varName)
    : raw_subarray_type(con, nullptr, varName), storage_mixin(*this) {}

  ArrayReference(Thread_Safe_Context &con,
      Knowledge_Update_Settings *settings, const std::string &varName)
    : raw_subarray_type(con, settings, varName), storage_mixin(*this) {}

  template<typename... Args>
  ArrayReference(Thread_Safe_Context &con,
        Knowledge_Update_Settings *settings, const std::string &varName,
        unsigned int default_dim, Args... args)
    : raw_subarray_type(con, settings, varName, args...), storage_mixin(*this, default_dim) { }

  template<typename... Args>
  ArrayReference(Thread_Safe_Context &con, const std::string &varName,
        unsigned int default_dim, Args... args)
    : ArrayReference(con, nullptr, varName, default_dim, args...) {}

  template<typename... Args>
  ArrayReference(Knowledge_Base &kbase, Args... args)
    : ArrayReference(kbase.get_context(), args...) {}

#else
  ArrayReference(Thread_Safe_Context &con, const std::string &varName,
      unsigned int i0 = 0, unsigned int i1 = 0, unsigned int i2 = 0, unsigned int i3 = 0,
      unsigned int i4 = 0, unsigned int i5 = 0, unsigned int i6 = 0, unsigned int i7 = 0,
      unsigned int i8 = 0, unsigned int i9 = 0) :
    raw_subarray_type(con, nullptr, varName,
      i1, i2, i3, i4, i5, i6, i7, i8, i9), storage_mixin(*this, i0) { }

  ArrayReference(Thread_Safe_Context &con,
      Knowledge_Update_Settings *settings, const std::string &varName,
      unsigned int i0 = 0, unsigned int i1 = 0, unsigned int i2 = 0, unsigned int i3 = 0,
      unsigned int i4 = 0, unsigned int i5 = 0, unsigned int i6 = 0, unsigned int i7 = 0,
      unsigned int i8 = 0, unsigned int i9 = 0) :
    raw_subarray_type(con, settings, varName,
      i1, i2, i3, i4, i5, i6, i7, i8, i9), storage_mixin(*this, i0) { }

  ArrayReference(Knowledge_Base &kbase, const std::string &varName,
      unsigned int i0 = 0, unsigned int i1 = 0, unsigned int i2 = 0, unsigned int i3 = 0,
      unsigned int i4 = 0, unsigned int i5 = 0, unsigned int i6 = 0, unsigned int i7 = 0,
      unsigned int i8 = 0, unsigned int i9 = 0) :
    raw_subarray_type(kbase.get_context(), nullptr, varName,
      i1, i2, i3, i4, i5, i6, i7, i8, i9), storage_mixin(*this, i0) { }

  ArrayReference(Knowledge_Base &kbase,
      Knowledge_Update_Settings *settings, const std::string &varName,
      unsigned int i0 = 0, unsigned int i1 = 0, unsigned int i2 = 0, unsigned int i3 = 0,
      unsigned int i4 = 0, unsigned int i5 = 0, unsigned int i6 = 0, unsigned int i7 = 0,
      unsigned int i8 = 0, unsigned int i9 = 0) :
    raw_subarray_type(kbase.get_context(), settings, varName,
      i1, i2, i3, i4, i5, i6, i7, i8, i9), storage_mixin(*this, i0) { }
#endif

  index_type operator[](unsigned int i)
  {
    reference_type ret(*this);
#ifdef USE_RVAL_REF
    return std::move(std::move(ret)[i]);
#else
    return ret[i];
#endif
  }

#ifdef USE_VAR_TMPL
  template<unsigned int dNew0, unsigned int... dNewN>
  ArrayReference(const ArrayReference<T, dNew0, dNewN...> &o)
    : raw_subarray_type(o), storage_mixin(*this) {}
#endif

  template<typename A>
  friend class __INTERNAL__::ArrayReferenceReference;

  friend class __INTERNAL__::identity<sm_type>::type;

  template<typename A>
  friend class sm_type::BaseMixin;

  template<typename A>
  friend class sm_type::RefBaseMixin;

  template<typename A, unsigned int Size, unsigned int Dims>
  friend class sm_type::DimensionMixin;

  template<typename A>
  friend class sm_type::RefDimensionMixin;

#ifdef USE_VAR_TMPL
  template<typename X, unsigned int x0, unsigned int... xN>
  friend class ArrayReference;
#else
  template <typename X, unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3,
                        unsigned int x4, unsigned int x5, unsigned int x6, unsigned int x7,
                        unsigned int x8, unsigned int x9>
  friend class ArrayReference;
#endif

#ifdef USE_VAR_TMPL
  std::array<int, dims> get_static_dims() const
  {
    return std::array<int, dims>{d0, dN...};
  }
#endif
  std::vector<int> get_dims() const
  {
    std::vector<int> ret;
    get_dims(ret);
    return ret;
  }

protected:
  void get_dims(std::vector<int> &out) const
  {
    out.push_back(get_size<0>());
    subarray_type::get_dims(out);
  }
};

#ifdef USE_VAR_TMPL
template <class T>
class ArrayReference<T>
  : public __INTERNAL__::ArrayReferenceBase,
    public StorageManager::get_sm_info<T>::sm_type::template BaseMixin<ArrayReference<T> >
#else
template <class T>
class ArrayReference<T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0>
  : public __INTERNAL__::ArrayReferenceBase,
    public StorageManager::get_sm_info<T>::sm_type::template BaseMixin<ArrayReference<T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0> >
#endif
{
public:
#ifdef USE_VAR_TMPL
  typedef ArrayReference<T> this_type;
#else
  typedef ArrayReference<T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0> this_type;
#endif
  //typedef __INTERNAL__::ArrayReferenceBase subarray_type;
  typedef this_type subarray_type;
  const static unsigned int dims = 0;
  const static unsigned int static_size = 1;

  typedef T storage_specifier;

  typedef StorageManager::get_sm_info<T> sm_info;
  typedef typename sm_info::sm_type sm_type;
  typedef typename sm_info::data_type value_type;

  typedef typename sm_type::template BaseMixin<this_type> storage_mixin;

  typedef __INTERNAL__::ArrayReferenceReference<this_type> reference_type;

  friend class __INTERNAL__::identity<sm_type>::type;

  template<typename A>
  friend class sm_type::BaseMixin;

  template<typename A>
  friend class sm_type::RefBaseMixin;

  template<typename A, unsigned int Size, unsigned int Dims>
  friend class sm_type::DimensionMixin;

  template<typename A>
  friend class sm_type::RefDimensionMixin;

#ifdef USE_VAR_TMPL
  template<typename X, unsigned int x0, unsigned int... xN>
  friend class ArrayReference;
#else
  template <typename X, unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3,
                        unsigned int x4, unsigned int x5, unsigned int x6, unsigned int x7,
                        unsigned int x8, unsigned int x9>
  friend class ArrayReference;

#endif

  #if 0
  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const size_mgr &get_size_mgr() const
  {
    return static_cast<const size_mgr &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  size_mgr &get_size_mgr()
  {
    return static_cast<size_mgr &>(*this);
  }
  #endif
public:
#if defined(USE_STD_ARRAY) && defined(USE_USING_TYPE)
  template<class E = value_type>
  using get_array_type = E;
#else
  template<class E = value_type>
  struct get_array_type
  {
    typedef E type;
  };
#endif


#ifdef USE_USING_TYPE
  template<class E = value_type>
  using get_vector_type = E;
#endif
  template<class E = value_type>
  struct get_vector_type_compat
  {
    typedef E type;
  };

  template<unsigned int dimension, bool dummy = true>
  struct get_dimension_type
  {
    typedef typename get_dimension_type<dimension - 1, dummy>::type type;
  };

  template<bool dummy>
  struct get_dimension_type<0, dummy>
  {
    typedef this_type type;
  };

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  const typename get_dimension_type<dimension>::type::storage_mixin &get_storage_mixin() const
  {
    return static_cast<const typename get_dimension_type<dimension>::type::storage_mixin &>(*this);
  }

  template<unsigned int dimension
#ifdef USE_VAR_TMPL
  =0
#endif
  >
  typename get_dimension_type<dimension>::type::storage_mixin &get_storage_mixin()
  {
    return static_cast<typename get_dimension_type<dimension>::type::storage_mixin &>(*this);
  }

  template<unsigned int i>
  this_type &get_dimension()
  {
    return *this;
  }

  template<unsigned int i>
  const this_type &get_dimension() const
  {
    return *this;
  }

  template<unsigned int i>
  unsigned int get_size() const
  {
    return 1;
  }

  template<unsigned int i>
  unsigned int get_multiplier() const
  {
    return 1;
  }

  template<class E>
  void get_into(std::vector<E> &out, reference_type v)
  {
#ifdef USE_EMPLACE
    out.emplace_back(v);
#else
    out.push_back(v);
#endif
  }

  void get_into(value_type &out, reference_type v)
  {
    out = v;
  }

  template<class E>
  void push_all(E &in)
  {
    in.push();
  }

  template<class E>
  void pull_all(E &in)
  {
    in.pull();
  }

  template<class E>
  void pull_all_keep_local(E &in)
  {
    in.pull_keep_local();
  }

  void set_from(const value_type &in, reference_type v)
  {
    v.set(in);
  }

  void update_from(const value_type &in, reference_type v)
  {
    if(v.exists())
      v.set(in);
  }

  void mark_modified(reference_type in)
  {
    //std::cerr << "Base mark_modified " << in.get_name() << std::endl;
    in.mark_modified();
  }

  //typedef this_type forwarded_type;
public:

  ArrayReference
    (Thread_Safe_Context &con,
      Knowledge_Update_Settings *settings = nullptr, const std::string &varName = ""
#ifndef USE_VAR_TMPL
      , unsigned int i0 = 0, unsigned int i1 = 0, unsigned int i2 = 0, unsigned int i3 = 0,
      unsigned int i4 = 0, unsigned int i5 = 0, unsigned int i6 = 0, unsigned int i7 = 0,
      unsigned int i8 = 0, unsigned int i9 = 0
#endif
      ) : ArrayReferenceBase(con, settings, varName), storage_mixin(*this) {}
};

}
}
}

#include "StatelessStorage.hpp"
#include "LazyStorage.hpp"
#include "ProactiveStorage.hpp"

#endif
