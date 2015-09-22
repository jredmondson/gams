#include "gams/algorithms/Formation_Sync.h"
#include "gams/platforms/Debug_Platform.h"
#include "gams/loggers/Global_Logger.h"
#include "madara/knowledge_engine/Knowledge_Base.h"

namespace loggers = gams::loggers;
namespace engine = Madara::Knowledge_Engine;
namespace algorithms = gams::algorithms;
namespace variables = gams::variables;
namespace platforms = gams::platforms;
namespace transport = Madara::Transport;
namespace containers = engine::Containers;

void test_defaults (void)
{
  transport::QoS_Transport_Settings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::Knowledge_Base knowledge ("", settings);
  engine::Knowledge_Base knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge, 0);

  variables::Sensors sensors;
  variables::Devices devices;

  variables::init_vars (devices, knowledge, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  Madara::Knowledge_Vector args;
  args.push_back ("end");
  args.push_back (end);
  args.push_back ("start");
  args.push_back (start);

  platforms::Debug_Platform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::Debug_Platform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::Debug_Platform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::Debug_Platform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::Debug_Platform platform0 (&knowledge, &sensors, 0, &self0);

  algorithms::Formation_Sync_Factory factory;
  algorithms::Base_Algorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &devices);
  algorithms::Base_Algorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &devices);
  algorithms::Base_Algorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &devices);
  algorithms::Base_Algorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &devices);
  algorithms::Base_Algorithm * alg0 = factory.create (
    args, &knowledge, &platform0, &sensors, &self0, &devices);

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
  transport::QoS_Transport_Settings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::Knowledge_Base knowledge ("", settings);
  engine::Knowledge_Base knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge, 0);

  variables::Sensors sensors;
  variables::Devices devices;

  variables::init_vars (devices, knowledge, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  Madara::Knowledge_Vector args;
  args.push_back ("end");
  args.push_back (end);
  args.push_back ("start");
  args.push_back (start);
  args.push_back ("formation");
  args.push_back ("triangle");

  platforms::Debug_Platform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::Debug_Platform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::Debug_Platform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::Debug_Platform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::Debug_Platform platform0 (&knowledge, &sensors, 0, &self0);

  algorithms::Formation_Sync_Factory factory;
  algorithms::Base_Algorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &devices);
  algorithms::Base_Algorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &devices);
  algorithms::Base_Algorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &devices);
  algorithms::Base_Algorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &devices);
  algorithms::Base_Algorithm * alg0 = factory.create (
    args, &knowledge, &platform0, &sensors, &self0, &devices);

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


void test_groups (void)
{
  transport::QoS_Transport_Settings settings;
  settings.hosts.push_back ("239.255.0.1:4150");
  settings.type = transport::MULTICAST;

  engine::Knowledge_Base knowledge0 ("", settings);
  engine::Knowledge_Base knowledge1 ("", settings), knowledge2 ("", settings),
    knowledge3 ("", settings), knowledge4 ("", settings);

  containers::String_Vector group3_1 ("group.3.members", knowledge1);
  containers::String_Vector group3_2 ("group.3.members", knowledge2);
  containers::String_Vector group3_3 ("group.3.members", knowledge3);
  containers::String_Vector group3_4 ("group.3.members", knowledge4);
  containers::String_Vector group3_0 ("group.3.members", knowledge0);

  group3_0.push_back ("device.1");
  group3_0.push_back ("device.3");
  group3_0.push_back ("device.4");

  group3_1.push_back ("device.1");
  group3_1.push_back ("device.3");
  group3_1.push_back ("device.4");

  group3_2.push_back ("device.1");
  group3_2.push_back ("device.3");
  group3_2.push_back ("device.4");

  group3_3.push_back ("device.1");
  group3_3.push_back ("device.3");
  group3_3.push_back ("device.4");

  group3_4.push_back ("device.1");
  group3_4.push_back ("device.3");
  group3_4.push_back ("device.4");

  variables::Self self1, self2, self3, self4, self0;
  self1.init_vars (knowledge1, 1);
  self2.init_vars (knowledge2, 2);
  self3.init_vars (knowledge3, 3);
  self4.init_vars (knowledge4, 4);
  self0.init_vars (knowledge0, 0);

  variables::Sensors sensors;
  variables::Devices devices;

  variables::init_vars (devices, knowledge0, 5);

  std::vector <double> start, end;
  start.push_back (40.437024);
  start.push_back (-79.948909);

  end.push_back (40.436834);
  end.push_back (-79.947911);

  Madara::Knowledge_Vector args;
  args.push_back ("end");
  args.push_back (end);
  args.push_back ("start");
  args.push_back (start);
  args.push_back ("group");
  args.push_back ("3");

  platforms::Debug_Platform platform1 (&knowledge1, &sensors, 0, &self1);
  platforms::Debug_Platform platform2 (&knowledge2, &sensors, 0, &self2);
  platforms::Debug_Platform platform3 (&knowledge3, &sensors, 0, &self3);
  platforms::Debug_Platform platform4 (&knowledge4, &sensors, 0, &self4);
  platforms::Debug_Platform platform0 (&knowledge0, &sensors, 0, &self0);

  algorithms::Formation_Sync_Factory factory;
  algorithms::Base_Algorithm * alg1 = factory.create (
    args, &knowledge1, &platform1, &sensors, &self1, &devices);
  algorithms::Base_Algorithm * alg2 = factory.create (
    args, &knowledge2, &platform2, &sensors, &self2, &devices);
  algorithms::Base_Algorithm * alg3 = factory.create (
    args, &knowledge3, &platform3, &sensors, &self3, &devices);
  algorithms::Base_Algorithm * alg4 = factory.create (
    args, &knowledge4, &platform4, &sensors, &self4, &devices);

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
  Madara::Knowledge_Record::set_precision (6);
  loggers::global_logger->set_level (loggers::LOG_DETAILED);

  test_triangle ();
  test_defaults ();
  test_groups ();

  return 0;
}
