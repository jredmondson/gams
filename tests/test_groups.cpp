
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Map.h"
#include "madara/knowledge/containers/StringVector.h"

#include "gams/loggers/GlobalLogger.h"
#include "gams/groups/GroupTransient.h"
#include "gams/groups/GroupFixedList.h"
#include "gams/groups/GroupFactoryRepository.h"

namespace loggers = gams::loggers;
namespace knowledge = madara::knowledge;
namespace transport = madara::transport;
namespace containers = knowledge::containers;
namespace groups = gams::groups;

int gams_fails = 0;

void test_fixed_list (knowledge::KnowledgeBase & knowledge)
{
  loggers::global_logger->log (
    0, "Testing GroupFixedList\n");

  groups::AgentVector new_members, orig_members;
  groups::AgentVector copy_members;
  containers::StringVector member_list;
  containers::Integer type;

  new_members.push_back ("agent.0");
  new_members.push_back ("leader.0");

  groups::GroupFixedList group_original, group_copy;
  group_original.add_members (new_members);
  group_original.get_members (orig_members);

  group_original.write ("group.fixed", &knowledge);

  // check the information in group.fixed
  member_list.set_name ("group.fixed.members", knowledge);
  type.set_name ("group.fixed.type", knowledge);

  group_copy.set_prefix ("group.fixed", &knowledge);
  group_copy.sync ();
  group_copy.get_members (copy_members);

  if (type == groups::GROUP_FIXED_LIST)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: knowledge base holds correct type info\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: knowledge base holds incorrect type info: %d\n",
      (int)*type);
    ++gams_fails;
  }

  if (member_list.size () == 2 &&
    member_list[0] == "agent.0" && member_list[1] == "leader.0")
  {
    loggers::global_logger->log (
      0, "  SUCCESS: knowledge base holds correct member information\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: Member information in GroupFixedList was incorrect\n");
    loggers::global_logger->log (
      0, "    member_list.size () was %d (correct is 2)\n");

    for (size_t i = 0; i < member_list.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    member_list[%d] was %s\n", (int)i, member_list[i].c_str ());
    }
    ++gams_fails;
  }

  if (new_members == orig_members)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: get_members function returned correct members\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: get_members function did not return correct members\n");

    for (size_t i = 0; i < orig_members.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    orig_members[%d] was %s\n", (int)i, orig_members[i].c_str ());
    }
    ++gams_fails;
  }

  if (copy_members == orig_members)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: copy get_members function returned correct members\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: copy get_members function did not have correct members\n");

    for (size_t i = 0; i < copy_members.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    copy_members[%d] was %s\n", (int)i, copy_members[i].c_str ());
    }
    ++gams_fails;
  }
}

void test_transient (knowledge::KnowledgeBase & knowledge)
{
  loggers::global_logger->log (
    0, "Testing GroupTransient\n");

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

  group_original.write ("group.transient", &knowledge);

  // check the information in group.fixed
  member_list.set_name ("group.transient.members", knowledge);
  type.set_name ("group.transient.type", knowledge);

  group_copy.set_prefix ("group.transient", &knowledge);
  group_copy.sync ();
  group_copy.get_members (copy_members);

  if (type == groups::GROUP_TRANSIENT)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: knowledge base holds correct type info\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: knowledge base holds incorrect type info: %d\n",
      (int)*type);
    ++gams_fails;
  }

  if (member_list.size () == 3 &&
    member_list.exists ("agent.0") && member_list.exists ("agent.1") &&
    member_list.exists ("leader.0"))
  {
    loggers::global_logger->log (
      0, "  SUCCESS: knowledge base holds correct member information\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: Member information in GroupFixedList was incorrect\n");
    loggers::global_logger->log (
      0, "    member_list.size () was %d (correct is 3)\n");

    for (size_t i = 0; i < orig_members.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    member_list[%d] was %s\n", (int)i, orig_members[i].c_str ());
    }
    ++gams_fails;
  }

  if (new_members == orig_members)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: get_members function returned correct members\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: get_members function did not return correct members\n");

    for (size_t i = 0; i < orig_members.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    orig_members[%d] was %s\n", (int)i, orig_members[i].c_str ());
    }
    ++gams_fails;
  }

  if (copy_members == orig_members)
  {
    loggers::global_logger->log (
      0, "  SUCCESS: copy get_members function returned correct members\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: copy get_members function did not have correct members\n");

    for (size_t i = 0; i < copy_members.size (); ++i)
    {
      loggers::global_logger->log (
        0, "    copy_members[%d] was %s\n", (int)i, copy_members[i].c_str ());
    }
    ++gams_fails;
  }
}

void test_repository (knowledge::KnowledgeBase & knowledge)
{
  loggers::global_logger->log (
    0, "Testing GroupFactoryRepository\n");

  groups::GroupBase * result (0);
  groups::GroupFactoryRepository repository (&knowledge);

  result = repository.create ("group.transient");

  if (dynamic_cast <groups::GroupTransient *> (result))
  {
    loggers::global_logger->log (
      0, "  SUCCESS: GroupTransient was successfully created\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: GroupTransient creation was unsuccessful\n");
    ++gams_fails;
  }

  result = repository.create ("group.fixed");

  if (dynamic_cast <groups::GroupFixedList *> (result))
  {
    loggers::global_logger->log (
      0, "  SUCCESS: GroupFixedList was successfully created\n");
  }
  else
  {
    loggers::global_logger->log (
      0, "  FAIL: GroupFixedList creation was unsuccessful\n");
    ++gams_fails;
  }
}

int main(int , char **)
{
  knowledge::KnowledgeRecord::set_precision (6);
  loggers::global_logger->set_level (loggers::LOG_DETAILED);

  knowledge::KnowledgeBase knowledge;

  test_fixed_list (knowledge);
  test_transient (knowledge);
  test_repository (knowledge);

  knowledge.print ();

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
