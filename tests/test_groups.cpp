
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Map.h"
#include "madara/knowledge/containers/StringVector.h"

#include "gams/loggers/GlobalLogger.h"
#include "gams/groups/GroupTransient.h"
#include "gams/groups/GroupFixedList.h"
#include "gams/groups/GroupFactoryRepository.h"
#include "gtest/gtest.h"

namespace loggers = gams::loggers;
namespace knowledge = madara::knowledge;
namespace transport = madara::transport;
namespace containers = knowledge::containers;
namespace groups = gams::groups;

static knowledge::KnowledgeBase knowledge_instance;

TEST (TestGroups, TestFixedList)
{
  groups::AgentVector new_members, orig_members;
  groups::AgentVector copy_members;
  containers::StringVector member_list;
  containers::Integer type;

  new_members.push_back ("agent.0");
  new_members.push_back ("leader.0");

  groups::GroupFixedList group_original, group_copy;
  group_original.add_members (new_members);
  group_original.get_members (orig_members);

  group_original.write ("group.fixed", &knowledge_instance);

  // check the information in group.fixed
  member_list.set_name ("group.fixed.members", knowledge_instance);
  type.set_name ("group.fixed.type", knowledge_instance);

  group_copy.set_prefix ("group.fixed", &knowledge_instance);
  group_copy.sync ();
  group_copy.get_members (copy_members);

  EXPECT_EQ(type, groups::GROUP_FIXED_LIST) << "Knowledge base doesn't hold correct type info";

  EXPECT_EQ(member_list.size(), 2) << "Knowledge base doesn't hold correct member info.";
  EXPECT_EQ(member_list[0], "agent.0") << "Knowledge base doesn't hold correct member info.";
  EXPECT_EQ(member_list[1], "leader.0") << "Knowledge base doesn't hold correct member info.";

  EXPECT_TRUE(new_members == orig_members) << "get_members didn't return correct members";
  EXPECT_TRUE(copy_members == orig_members) << "copy get_members didn't return correct members";
}

TEST(TestGroups, TestTransient)
{
  groups::AgentVector new_members, orig_members;
  groups::AgentVector copy_members;
  containers::Map member_list;
  containers::Integer type;

  new_members.push_back ("agent.0");
  new_members.push_back ("agent.1");
  new_members.push_back ("leader.0");

  groups::GroupTransient group_original, group_copy;
  group_original.add_members (new_members);
  group_original.get_members (orig_members);

  group_original.write ("group.transient", &knowledge_instance);

  // check the information in group.fixed
  member_list.set_name ("group.transient.members", knowledge_instance);
  type.set_name ("group.transient.type", knowledge_instance);

  group_copy.set_prefix ("group.transient", &knowledge_instance);
  group_copy.sync ();
  group_copy.get_members (copy_members);

  EXPECT_EQ(type, groups::GROUP_TRANSIENT) << "Knowledge base doesn't hold correct type info";

  EXPECT_EQ(member_list.size(), 3) << "Knowledge base doesn't hold correct member info.";
  EXPECT_TRUE(member_list.exists("agent.0")) << "Knowledge base doesn't hold correct member info.";
  EXPECT_TRUE(member_list.exists("agent.1")) << "Knowledge base doesn't hold correct member info.";
  EXPECT_TRUE(member_list.exists("leader.0"))<< "Knowledge base doesn't hold correct member info.";

  EXPECT_TRUE(new_members == orig_members) << "get_members didn't return correct members";

  // TODO: this one is flaky, probably because of shadowing GroupBase::knowledge_
  // inside GroupTransient
  EXPECT_TRUE(copy_members == orig_members) << "copy get_members didn't return correct members";
}

TEST(TestGroups, TestRepository)
{
  groups::GroupBase * result (0);
  groups::GroupFactoryRepository repository (&knowledge_instance);

  result = repository.create ("group.transient");

  EXPECT_TRUE(dynamic_cast <groups::GroupTransient *> (result) != nullptr)
      << "GroupTransient failed to create from repository";

  result = repository.create ("group.fixed");

  EXPECT_TRUE(dynamic_cast <groups::GroupFixedList *> (result) != nullptr)
      << "GroupFixedList failed to create from repository";
}

int main(int argc, char** argv)
{
  knowledge::KnowledgeRecord::set_precision (6);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
