#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunused-local-typedef"
#endif

#include <boost/python/detail/wrap_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/import.hpp>
#include <boost/python/enum.hpp>

#include "gams/pose/ReferenceFrame.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "FunctionDefaults.h"
#include "GamsPose.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

/**
 * @file Madara.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains Boost.Python mappings for the C++ MADARA library
 * to a Python module called madara.
 **/

using namespace boost::python;

template<typename Container>
list stl_to_python(const Container& input)
{
  typedef typename Container::value_type T;

  list lst;
  std::for_each(input.begin(), input.end(), [&](const T& t) { lst.append(t); });

  return lst;
}

template<typename Container>
Container python_to_stl(const list& input)
{
  typedef typename Container::value_type T;

  Container vec;
  stl_input_iterator<T> beg(input), end;
  std::for_each(beg, end, [&](const T& t) { vec.push_back(t); });

  return vec;
}

BOOST_PYTHON_MODULE(gams)
{
  // Launch the appropriate thread management initialization
  PyEval_InitThreads();

  // Declare classes inside Madara namespace (top namespace of Python module)

  class_<std::vector<std::string>>("StringVector")
      .def(vector_indexing_suite<std::vector<std::string>>());

  class_<std::vector<gams::pose::ReferenceFrame>>("FrameVector")
      .def(vector_indexing_suite<std::vector<gams::pose::ReferenceFrame>>());

  def("to_pyframes", &stl_to_python<std::vector<gams::pose::ReferenceFrame>>);

  def("from_pyframes", &python_to_stl<std::vector<gams::pose::ReferenceFrame>>);

  define_pose();
}
