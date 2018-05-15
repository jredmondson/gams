#include <iostream>
#include <math.h>
#include <gams/CPP11_compat.h>
#include <gams/utility/ArgumentParser.h>

namespace knowledge = madara::knowledge;
using namespace gams::utility;

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

  for(ArgumentParser::const_iterator i = args.begin();
      i != args.end(); i.next())
  {
    std::cout << i.name() << " -> " << i.value() << std::endl;
  }

  std::cout << "With dereference:" << std::endl;

  for(ArgumentParser::const_iterator i = args.begin(); i != args.end(); ++i)
  {
    std::cout << i->first << " -> " << i->second << std::endl;
  }

#ifdef CPP11
  std::cout << "With C++11 foreach:" << std::endl;

  for(const auto &i : args)
  {
    std::cout << i.first << " -> " << i.second << std::endl;
  }
#endif

  return 0;
}
