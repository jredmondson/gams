#include "gams/algorithms/FormationSync.h"
#include "gams/platforms/DebugPlatform.h"
#include "gams/loggers/GlobalLogger.h"
#include "madara/knowledge/KnowledgeBase.h"

namespace loggers = gams::loggers;
namespace engine = madara::knowledge;
namespace algorithms = gams::algorithms;
namespace variables = gams::variables;
namespace platforms = gams::platforms;
namespace transport = madara::transport;
namespace containers = engine::containers;

void test_defaults (void)
{
  transport::QoSTransportSettings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::KnowledgeBase knowledge ("", settings);
  engine::KnowledgeBase knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge, 0);

  variables::Sensors sensors;
  variables::Agents agents;

  variables::init_vars (agents, knowledge, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  madara::knowledge::KnowledgeMap args;
  args["end"] = end;
  args["start"] = start;

  platforms::DebugPlatform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::DebugPlatform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::DebugPlatform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::DebugPlatform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::DebugPlatform platform0 (&knowledge, &sensors, 0, &self0);

  algorithms::FormationSyncFactory factory;
  algorithms::BaseAlgorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &agents);
  algorithms::BaseAlgorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &agents);
  algorithms::BaseAlgorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &agents);
  algorithms::BaseAlgorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &agents);
  algorithms::BaseAlgorithm * alg0 = factory.create (
    args, &knowledge, &platform0, &sensors, &self0, &agents);

  for (int i = 0; i < 24; ++i)
  {
    alg1->analyze ();
    alg1->execute ();
    knowledge1.send_modifieds ();

    alg2->analyze ();
    alg2->execute ();
    knowledge2.send_modifieds ();

    alg3->analyze ();

    alg3->execute ();
    knowledge3.send_modifieds ();

    alg4->analyze ();
    alg4->execute ();
    knowledge4.send_modifieds ();

    alg0->analyze ();
    alg0->execute ();
    knowledge.send_modifieds ();

    knowledge.print ();
    knowledge1.print ();
    knowledge2.print ();
    knowledge3.print ();
    knowledge4.print ();
  }

  loggers::global_logger->log (0, "  Finished default formation sync");
}


void test_triangle (void)
{
  transport::QoSTransportSettings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::KnowledgeBase knowledge ("", settings);
  engine::KnowledgeBase knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge, 0);

  variables::Sensors sensors;
  variables::Agents agents;

  variables::init_vars (agents, knowledge, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  madara::knowledge::KnowledgeMap args;
  args["end"] = (end);
  args["start"] = (start);
  args["formation"] = ("triangle");

  platforms::DebugPlatform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::DebugPlatform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::DebugPlatform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::DebugPlatform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::DebugPlatform platform0 (&knowledge, &sensors, 0, &self0);

  algorithms::FormationSyncFactory factory;
  algorithms::BaseAlgorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &agents);
  algorithms::BaseAlgorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &agents);
  algorithms::BaseAlgorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &agents);
  algorithms::BaseAlgorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &agents);
  algorithms::BaseAlgorithm * alg0 = factory.create (
    args, &knowledge, &platform0, &sensors, &self0, &agents);

  for (int i = 0; i < 24; ++i)
  {
    alg1->analyze ();
    alg1->execute ();
    knowledge1.send_modifieds ();

    alg2->analyze ();
    alg2->execute ();
    knowledge2.send_modifieds ();

    alg3->analyze ();

    alg3->execute ();
    knowledge3.send_modifieds ();

    alg4->analyze ();
    alg4->execute ();
    knowledge4.send_modifieds ();

    alg0->analyze ();
    alg0->execute ();
    knowledge.send_modifieds ();

    knowledge.print ();
    knowledge1.print ();
    knowledge2.print ();
    knowledge3.print ();
    knowledge4.print ();
  }

  loggers::global_logger->log (0, "  Finished triangle formation sync");
}


void test_rectangle (void)
{
  transport::QoSTransportSettings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::KnowledgeBase knowledge ("", settings);
  engine::KnowledgeBase knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge, 0);

  variables::Sensors sensors;
  variables::Agents agents;

  variables::init_vars (agents, knowledge, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  madara::knowledge::KnowledgeMap args;
  args["end"] = (end);
  args["start"] = (start);
  args["formation"] = ("rectangle");

  platforms::DebugPlatform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::DebugPlatform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::DebugPlatform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::DebugPlatform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::DebugPlatform platform0 (&knowledge, &sensors, 0, &self0);

  algorithms::FormationSyncFactory factory;
  algorithms::BaseAlgorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &agents);
  algorithms::BaseAlgorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &agents);
  algorithms::BaseAlgorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &agents);
  algorithms::BaseAlgorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &agents);
  algorithms::BaseAlgorithm * alg0 = factory.create (
    args, &knowledge, &platform0, &sensors, &self0, &agents);

  for (int i = 0; i < 24; ++i)
  {
    alg1->analyze ();
    alg1->execute ();
    knowledge1.send_modifieds ();

    alg2->analyze ();
    alg2->execute ();
    knowledge2.send_modifieds ();

    alg3->analyze ();

    alg3->execute ();
    knowledge3.send_modifieds ();

    alg4->analyze ();
    alg4->execute ();
    knowledge4.send_modifieds ();

    alg0->analyze ();
    alg0->execute ();
    knowledge.send_modifieds ();

    knowledge.print ();
    knowledge1.print ();
    knowledge2.print ();
    knowledge3.print ();
    knowledge4.print ();
  }

  loggers::global_logger->log (0, "  Finished rectangle formation sync");
}


void test_groups (void)
{
  transport::QoSTransportSettings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::KnowledgeBase knowledge0 ("", settings);
  engine::KnowledgeBase knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  containers::StringVector group3_1 ("group.3.members", knowledge1);
  containers::StringVector group3_2 ("group.3.members", knowledge2);
  containers::StringVector group3_3 ("group.3.members", knowledge3);
  containers::StringVector group3_4 ("group.3.members", knowledge4);
  containers::StringVector group3_0 ("group.3.members", knowledge0);

  group3_0.push_back ("agent.1");
  group3_0.push_back ("agent.3");
  group3_0.push_back ("agent.4");

  group3_1.push_back ("agent.1");
  group3_1.push_back ("agent.3");
  group3_1.push_back ("agent.4");

  group3_2.push_back ("agent.1");
  group3_2.push_back ("agent.3");
  group3_2.push_back ("agent.4");

  group3_3.push_back ("agent.1");
  group3_3.push_back ("agent.3");
  group3_3.push_back ("agent.4");

  group3_4.push_back ("agent.1");
  group3_4.push_back ("agent.3");
  group3_4.push_back ("agent.4");

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge0, 0);

  variables::Sensors sensors;
  variables::Agents agents;

  variables::init_vars (agents, knowledge0, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  madara::knowledge::KnowledgeMap args;
  args["end"] = (end);
  args["start"] = (start);
  args["group"] = ("3");

  platforms::DebugPlatform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::DebugPlatform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::DebugPlatform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::DebugPlatform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::DebugPlatform platform0 (&knowledge0, &sensors, 0, &self0);

  algorithms::FormationSyncFactory factory;
  algorithms::BaseAlgorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &agents);
  algorithms::BaseAlgorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &agents);
  algorithms::BaseAlgorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &agents);
  algorithms::BaseAlgorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &agents);

  for (int i = 0; i < 24; ++i)
  {
    alg1->analyze ();
    alg1->execute ();
    knowledge1.send_modifieds ();

    alg2->analyze ();
    alg2->execute ();
    knowledge2.send_modifieds ();

    alg3->analyze ();
    alg3->execute ();
    knowledge3.send_modifieds ();

    alg4->analyze ();
    alg4->execute ();
    knowledge4.send_modifieds ();

    knowledge0.print ();
    knowledge1.print ();
    knowledge2.print ();
    knowledge3.print ();
    knowledge4.print ();
  }

  loggers::global_logger->log (0, "  Finished group default formation sync");
}

int main(int argc, char *argv[])
{
  madara::knowledge::KnowledgeRecord::set_precision (6);
  loggers::global_logger->set_level (loggers::LOG_DETAILED);

  test_rectangle ();
  test_triangle ();
  test_defaults ();
  test_groups ();

  return 0;
}
