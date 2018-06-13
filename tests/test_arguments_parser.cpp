#include <iostream>
#include <math.h>
#include <gams/CPP11_compat.h>
#include <gams/utility/ArgumentParser.h>
#include <map>
#include <string>
#include "gtest/gtest.h"

namespace knowledge = madara::knowledge;
using namespace gams::utility;

TEST(TestArgumentParser, TestParsing)
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

  std::map<std::string, std::string> expected = {
      {"aname", "a val"},
      {"bname", "b val"},
      {"cname", "c val"},
      {"dname", "d val"},
      {"size", "2"}
  };

  knowledge::KnowledgeMap kmap(kbase.to_map_stripped("args."));

  ArgumentParser args(kmap);

  for(ArgumentParser::const_iterator i = args.begin();
      i != args.end(); i.next())
  {
    auto val = expected[i.name()];
    EXPECT_EQ(val, i.value());
  }

  for(ArgumentParser::const_iterator i = args.begin(); i != args.end(); ++i)
  {
    auto val = expected[i->first];
    EXPECT_EQ(val, i->second);
  }

#ifdef CPP11
  for(const auto &i : args)
  {
    auto val = expected[i.first];
    EXPECT_EQ(val, i.second);
  }
#endif
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
