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

#ifndef _MADARA_CONTAINER_HPP
#define _MADARA_CONTAINER_HPP

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <climits>
#include <sstream>
#include <typeinfo>
#include <exception>
#include <madara/knowledge_engine/Thread_Safe_Context.h>
#include <madara/knowledge_engine/Thread_Safe_Context.h>
#include <madara/knowledge_engine/Knowledge_Update_Settings.h>
#include "knowledge_cast.hpp"

#if __cplusplus >= 201103L
#define USE_CPP11
#define USE_RVAL_REF
#define USE_VAR_TMPL
#define USE_STD_ARRAY
#define USE_DELETE_MEMBER
#endif

namespace Madara
{

namespace Knowledge_Engine
{
namespace Containers
{

namespace __INTERNAL__
{

template<typename T>
struct identity
{
  typedef T type;
};

template<typename T, typename Impl>
class BaseReference
{
protected:
  BaseReference() {}
  Impl* get_impl()
  {
    return static_cast<Impl*>(this);
  }

  const Impl* get_impl() const
  {
    return static_cast<const Impl*>(this);
  }
public:

  std::string get_name() const
  {
    return get_impl()->get_name();
  }

  Thread_Safe_Context &get_context() const
  {
    return get_impl()->get_context();
  }

  /// Returns previous settings
  Knowledge_Update_Settings *set_settings(Knowledge_Update_Settings *new_settings)
  {
    return get_impl()->set_settings(new_settings);;
  }

  Knowledge_Update_Settings *get_settings() const
  {
    return get_impl()->get_settings();;
  }

  const Knowledge_Update_Settings &get_settings_cref() const
  {
    return get_impl()->get_settings_cref();
    //return settings ? *settings : default_knowledge_settings;
  }

  Knowledge_Record get_knowledge_record() const {
    return get_impl()->get_knowledge_record();
  }

  void mark_modified()
  {
    get_impl()->mark_modified();
  }

  T get() const
  {
    return get_impl()->get();
    //return knowledge_cast<T>(get_knowledge_record());
  }

  operator T() const {
    //return get();
    T ret = get();
    //std::cout << "Getting " << get_name() << ": " << ret << std::endl;
    return ret;
  }

  bool exists() const
  {
    return get_context().exists(get_name(), get_settings_cref());
  }

  Knowledge_Record::Integer get_integer() const
  {
    return get_knowledge_record().to_integer();
  }

  double get_double() const
  {
    return get_knowledge_record().to_double();
  }

  std::string get_string() const
  {
    return get_knowledge_record().get_string();
  }

  template<typename R>
  R get_as() const
  {
    return knowledge_cast<R>(get_knowledge_record());
  }

  const T &operator=(const BaseReference &in)
  {
    return set(in.get());
  }

  template<class E>
  const T &operator=(const BaseReference<T, E> &in)
  {
    return this->set(in.get());
  }

  const T &operator=(const T& in)
  {
    //std::cout << "Setting " << get_name() << " to " << in << std::endl;
    return set(in);
  }

  const T &operator+=(const T& in)
  {
    return set(get() + in);
  }

  const T &operator-=(const T& in)
  {
    return set(get() - in);
  }

  const T &operator*=(const T& in)
  {
    return set(get() * in);
  }

  const T &operator/=(const T& in)
  {
    return set(get() / in);
  }

  const T &operator%=(const T& in)
  {
    return set(get() % in);
  }

  const T &operator|=(const T& in)
  {
    return set(get() | in);
  }

  const T &operator&=(const T& in)
  {
    return set(get() & in);
  }

  const T &operator^=(const T& in)
  {
    return set(get() ^ in);
  }

  template<typename I>
  const T &operator<<=(const I& in)
  {
    return set(get() << in);
  }

  template<typename I>
  const T &operator>>=(const I& in)
  {
    return set(get() << in);
  }

  const T &set(const T& in)
  {
    return set(in, get_settings_cref());
  }

  const T &set(const T& in, const Knowledge_Update_Settings &settings)
  {
    return get_impl()->set(in, settings);
  }

  const Knowledge_Record &set_knowledge_record(const Knowledge_Record &in)
  {
    return set_knowledge_record(in, get_settings_cref());
  }

  const Knowledge_Record &set_knowledge_record(const Knowledge_Record &in, const Knowledge_Update_Settings &settings)
  {
    return get_impl()->set_knowledge_record(in, settings);
  }
};

namespace
{
  /* All defaults, except don't expand variable names (siginificant performance penalty) */
  const Knowledge_Update_Settings default_knowledge_settings(false, true, false, false, false, 1);
}

class ContextHost
{
protected:
  Thread_Safe_Context &context;
  Knowledge_Update_Settings *settings;

  ContextHost(Knowledge_Base &kbase)
    : context(kbase.get_context()), settings() {}

  ContextHost(Thread_Safe_Context &con)
    : context(con), settings() {}

  ContextHost(Thread_Safe_Context &con, Knowledge_Update_Settings *settings)
    : context(con), settings(settings) {}

  ContextHost(Knowledge_Base &kbase, Knowledge_Update_Settings *settings)
    : context(kbase.get_context()), settings(settings) {}

  ContextHost(const ContextHost &o)
#ifdef USE_RVAL_REF
noexcept
#endif
    : context(o.get_context()), settings(o.get_settings()) {}

public:
  Thread_Safe_Context &get_context() const
  {
    return context;
  }

  /// Returns previous settings
  Knowledge_Update_Settings *set_settings(Knowledge_Update_Settings *new_settings)
  {
    Knowledge_Update_Settings *old_settings = settings;
    settings = new_settings;
    return old_settings;
  }

  Knowledge_Update_Settings *get_settings() const
  {
    return settings;
  }

  const Knowledge_Update_Settings &get_settings_cref() const
  {
    return settings ? *settings : default_knowledge_settings;
  }
};

}

template<typename T>
class Reference : public __INTERNAL__::BaseReference<T, Reference<T> >,
                  public __INTERNAL__::ContextHost
{
protected:
  typedef __INTERNAL__::BaseReference<T, Reference<T> > Base;
  typedef __INTERNAL__::ContextHost ContextStorage;

#ifdef USE_CPP11
  const Variable_Reference var_ref;
#else
  // to support putting Reference in a vector, pre-C++11, must be assignable
  Variable_Reference var_ref;
#endif

public:
  Reference(const Reference &o)
#ifdef USE_RVAL_REF
noexcept
#endif
    : ContextStorage(o), var_ref(o.var_ref) { }

#ifdef USE_RVAL_REF
  Reference(Reference &&o) noexcept
    : ContextStorage(std::move(o)), var_ref(std::move(o.var_ref)) {}
#endif

  Reference(Thread_Safe_Context &con, const std::string &name)
    : ContextStorage(con), var_ref(con.get_ref(name)) {}

  Reference(Knowledge_Base &kbase, const std::string &name)
    : ContextStorage(kbase), var_ref(this->get_context().get_ref(name)) {}

  Reference(Thread_Safe_Context &con, Knowledge_Update_Settings *settings, const std::string &name)
    : ContextStorage(con, settings), var_ref(con.get_ref(name, settings)) {}

  Reference(Knowledge_Base &kbase, Knowledge_Update_Settings *settings, const std::string &name)
    : ContextStorage(kbase, settings), var_ref(this->get_context().get_ref(name, settings)) {}

  template<typename Impl>
  Reference(const __INTERNAL__::BaseReference<T, Impl> &o)
    : ContextStorage(o.get_context(), o.get_settings()), var_ref(o.get_context().get_ref(o.get_name()))
  {
    //std::cerr << "Converting to Reference type from " << typeid(Impl).name() << std::endl;
  }

  using ContextStorage::get_settings;
  using ContextStorage::get_settings_cref;
  using ContextStorage::get_context;

  const T &operator=(const Reference &in)
  {
    return set(in.template get(), this->get_settings_cref());
  }

  template<typename Q>
  const T &operator=(const __INTERNAL__::BaseReference<T, Q> &in)
  {
    return set(in.get(), this->get_settings_cref());
  }

  std::string get_name() const
  {
    // const_cast required to workaround missing const decorator;
    // current implementation is actually const
    return std::string(const_cast<Variable_Reference&>(this->var_ref).get_name());
  }

  void mark_modified()
  {
    this->get_context().mark_modified(var_ref);
    //this->get_context().set(var_ref, 
  }

  Knowledge_Record get_knowledge_record() const {
    return this->get_context().get(var_ref, this->get_settings_cref());
  }

  T get() const
  {
    return knowledge_cast<T>(get_knowledge_record());
  }

  const Knowledge_Record &set_knowledge_record(const Knowledge_Record &in, const Knowledge_Update_Settings &settings)
  {
    this->get_context().set(var_ref, in, settings);
    return in;
  }

  const T &set(const T& in, const Knowledge_Update_Settings &settings)
  {
    set_knowledge_record(knowledge_cast(in), settings);
    return in;
  }

  using Base::operator=;
  using Base::set;
  using Base::set_knowledge_record;
};

template<typename T>
class CachedReference : public __INTERNAL__::BaseReference<T, CachedReference<T> >,
                        public __INTERNAL__::ContextHost
{
protected:
  typedef __INTERNAL__::BaseReference<T, CachedReference<T> > Base;
  typedef __INTERNAL__::ContextHost ContextStorage;

  struct data_t
  {
    const std::string name;
    bool exist:1;
    bool dirty:1;
    bool create:1;
    Variable_Reference var_ref;
    T data;
    mutable unsigned int ref_count;

    data_t(Thread_Safe_Context &con, const std::string &name) : name(name),
      exist(con.exists(name)), dirty(false), create(false),
      var_ref(exist ? con.get_ref(name) : Variable_Reference()),
      data(exist ? knowledge_cast<T>(con.get(name)) : T()),
      ref_count(1) {}

    data_t *new_ref()
#ifdef USE_RVAL_REF
noexcept
#endif
    {
      ++ref_count;
      return this;
    }

    const data_t &new_ref() const
#ifdef USE_RVAL_REF
noexcept
#endif
    {
      ++ref_count;
      return *this;
    }

    bool del_ref()
    {
      return ((--ref_count) == 0);
    }
  };

  data_t *data;
public:

  CachedReference(Thread_Safe_Context &con, const std::string &name)
    : ContextStorage(con), data(new data_t(con, name)) {}

  CachedReference(Knowledge_Base &kbase, const std::string &name)
    : ContextStorage(kbase), data(new data_t(kbase.get_context(), name)) {}

  CachedReference(Thread_Safe_Context &con, Knowledge_Update_Settings *settings, const std::string &name)
    : ContextStorage(con, settings), data(new data_t(con, name)) {}

  CachedReference(Knowledge_Base &kbase, Knowledge_Update_Settings *settings, const std::string &name)
    : ContextStorage(kbase, settings), data(new data_t(kbase.get_context(), name)) {}

  CachedReference(const CachedReference &o)
#ifdef USE_RVAL_REF
noexcept
#endif
    : ContextStorage(o), data(o.data->new_ref()) {
      //std::cerr << "Copying CachedReference" << std::endl;
    }
#ifdef USE_RVAL_REF
  CachedReference(CachedReference &&o) noexcept
    : ContextStorage(std::move(o)), data(o.data->new_ref()) {}
#endif

  template<typename Impl>
  CachedReference(const __INTERNAL__::BaseReference<T, Impl> &o)
    : ContextStorage(o.get_context(), o.get_settings()), data(new data_t(o.get_context(), o.get_name())) {
      //exist(this->get_context().exists(this->get_name(), this->get_settings())), dirty(false), create(false),
      //var_ref(exist ? this->get_context().get_ref(this->get_name(), this->get_settings()) : Variable_Reference()),
      //data(exist ? knowledge_cast<T>(this->get_context().get(this->get_name(), this->get_settings())) : T()) {
    //std::cerr << "Converting to CachedReference type from " << typeid(Impl).name() << std::endl;
  }

  ~CachedReference()
  {
    if(data->del_ref())
      delete data;
  }

  using ContextStorage::get_settings;
  using ContextStorage::get_settings_cref;
  using ContextStorage::get_context;

  std::string get_name() const
  {
    return data->name;
  }
  
  Knowledge_Record get_knowledge_record() const {
    return knowledge_cast(data->data);
  }

  const T &operator=(const CachedReference &in)
  {
    return set(in.template get(), this->get_settings_cref());
  }

  template<typename Q>
  const T &operator=(const __INTERNAL__::BaseReference<T, Q> &in)
  {
    return set(in.get(), this->get_settings_cref());
  }

  T get() const
  {
    return data->data;
  }

  const Knowledge_Record &set_knowledge_record(const Knowledge_Record &in, const Knowledge_Update_Settings &settings)
  {
    return set(knowledge_cast<T>(in), settings);
  }

  const T &set(const T& in, const Knowledge_Update_Settings &settings)
  {
    if(!data->exist)
    {
      data->exist = true;
      data->create = true;
      data->dirty = true;
      data->data = in;
    } else if(in != data->data)
    {
      data->dirty = true;
      data->data = in;
    }
    return data->data;
  }

  bool is_dirty()
  {
    return data->dirty;
  }

  void mark_modified()
  {
    data->dirty = true;
  }

protected:
  void ensure_exists()
  {
    if(data->create)
    {
      data->var_ref = this->get_context().get_ref(data->name, this->get_settings_cref());
      data->create = false;
    }
  }

public:
  void push()
  {
    if(is_dirty())
    {
      ensure_exists();
      this->get_context().set(data->var_ref, knowledge_cast(data->data), this->get_settings_cref());
      data->dirty = false;
    }
  }

  void pull()
  {
    ensure_exists();
    data->data = knowledge_cast<T>(this->get_context().get(data->var_ref, this->get_settings_cref()));
    data->dirty = false;
  }

  void pull_keep_local()
  {
    //std::cerr<<"pull_keep_local @ " << this->get_name() << ": dirty " << dirty << std::endl;
    if(is_dirty())
      pull();
  }

  using Base::operator=;
  using Base::set;
  using Base::set_knowledge_record;
};


}
}
}

#endif
