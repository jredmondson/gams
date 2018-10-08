#ifndef _MADARA_PYTHON_FUNCTION_DEFAULTS_
#define _MADARA_PYTHON_FUNCTION_DEFAULTS_

/**
 * @file FunctionDefaults.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains Boost.Python function overload definitions
 * for various MADARA functions.
 **/

#include "madara/knowledge/KnowledgeBase.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/args.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/class.hpp>
#include <boost/python/overloads.hpp>
#include <boost/python/return_internal_reference.hpp>

#include "madara/utility/Utility.h"

/********************************************************
 * Settings overloads
 ********************************************************/

/********************************************************
 * Class members (functions inside of classes)
 ********************************************************/

// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(m_to_string_0_of_1, to_string, 0, 1)

/********************************************************
 * Static functions (standalone functions)
 ********************************************************/

// BOOST_PYTHON_FUNCTION_OVERLOADS(
//     file_from_fragments_2_of_4, madara::utility::file_from_fragments, 2, 4)

#endif
