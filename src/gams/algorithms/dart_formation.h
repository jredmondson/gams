#ifndef   _GAMS_ALGORITHMS_DART_H_
#define   _GAMS_ALGORITHMS_DART_H_

#include "madara/knowledge_engine/Knowledge_Base.h"
#include "madara/knowledge_engine/Knowledge_Record.h"
#include "madara/knowledge_engine/Functions.h"
#include "madara/knowledge_engine/containers/Integer_Vector.h"
#include "madara/knowledge_engine/containers/Double_Vector.h"
#include "madara/transport/Packet_Scheduler.h"
#include "madara/threads/Threader.h"
#include "madara/filters/Generic_Filters.h"

#include "gams/controllers/Base_Controller.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/platforms/vrep/VREP_Base.h"
#include "gams/variables/Self.h"
#include "gams/utility/GPS_Position.h"
#include "gams/utility/Axes.h"

// begin dmpl namespace
namespace dmpl
{

namespace engine = Madara::Knowledge_Engine;
namespace threads = Madara::Threads;

namespace containers = engine::Containers;

namespace controllers = gams::controllers;

namespace platforms = gams::platforms;

namespace variables = gams::variables;
class GAMS_Export Algo_Factory : public ::gams::algorithms::Algorithm_Factory
{
public:

  virtual ::gams::algorithms::Base_Algorithm * create (
    const Madara::Knowledge_Vector & args,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge_,
    platforms::Base_Platform * platform,
    variables::Sensors * sensors,
    variables::Self * self,
    variables::Devices * devices);
};

class GAMS_Export SyncAlgo_Factory : public ::gams::algorithms::Algorithm_Factory
{
public:

  virtual ::gams::algorithms::Base_Algorithm * create (
    const Madara::Knowledge_Vector & args,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge_,
    platforms::Base_Platform * platform_,
    variables::Sensors * sensors,
    variables::Self * self,
    variables::Devices * devices);
};

} // end dmpl namespace

#endif
