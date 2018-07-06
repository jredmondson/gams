#include <iostream>
#include <math.h>
#include "gams/CPP11_compat.h"
#include "gams/utility/ArgumentParser.h"

namespace knowledge = madara::knowledge;
using namespace gams::utility;

int gams_fails = 0;

void test_name_value (ArgumentParser::const_iterator & iter, 
const ArgumentParser::const_iterator & end, const std::string & var, 
const std::string & value)
{
  if (iter != end)
  {
    if (iter.name () == var && iter.value () == value)
    {
      std::cerr << "SUCCESS: " << var << " = " << value << "\n";
    }
    else
    {
      std::cerr << "FAIL: " << iter.name () << " = " << iter.value () << " instead of"
        << var << " = " << value << "\n";
    }
    ++iter;
  }
  else
  {
    ++gams_fails;
    std::cerr << "FAIL: unexpected end of argument iteration\n";
  }
}

void test_deref_iteration (ArgumentParser::const_iterator & iter, 
const ArgumentParser::const_iterator & end, const std::string & var, 
const std::string & value)
{
  if (iter != end)
  {
    if (iter->first == var && iter->second == value)
    {
      std::cerr << "SUCCESS: " << var << " = " << value << "\n";
    }
    else
    {
      std::cerr << "FAIL: " << iter->first << " = " << iter->second << " instead of"
        << var << " = " << value << "\n";
    }
    ++iter;
  }
  else
  {
    ++gams_fails;
    std::cerr << "FAIL: unexpected end of argument iteration\n";
  }
}

int main(int, char *[])
{
  knowledge::KnowledgeBase kbase;
  kbase.set("not_args.asdf", "shouldn't appear");
  kbase.set("args.size", knowledge::KnowledgeRecord::Integer (2));
  kbase.set("args.0", "aname");
  kbase.set("args.1", "a val");
  kbase.set("args.2", "bname");
  kbase.set("args.3", "b val");
  kbase.set("args.cname", "c val");
  kbase.set("args.dname", "d val");

  knowledge::KnowledgeMap kmap(kbase.to_map_stripped("args."));

  ArgumentParser args(kmap);

  ArgumentParser::const_iterator ci = args.begin ();

  test_name_value (ci, args.end (), "aname", "a val");
  test_name_value (ci, args.end (), "bname", "b val");
  test_name_value (ci, args.end (), "cname", "c val");
  test_name_value (ci, args.end (), "dname", "d val");
  test_name_value (ci, args.end (), "size", "2");

  ci = args.begin ();

  test_deref_iteration (ci, args.end (), "aname", "a val");
  test_deref_iteration (ci, args.end (), "bname", "b val");
  test_deref_iteration (ci, args.end (), "cname", "c val");
  test_deref_iteration (ci, args.end (), "dname", "d val");
  test_deref_iteration (ci, args.end (), "size", "2");


#ifdef CPP11
  std::cout << "With C++11 foreach:" << std::endl;

  for(const auto &i : args)
  {
    std::cout << i.first << " -> " << i.second << std::endl;
  }
#endif

  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}
