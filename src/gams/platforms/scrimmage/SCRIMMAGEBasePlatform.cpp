#include "gams/platforms/scrimmage/SCRIMMAGEBasePlatform.h"
#include "scrimmage/entity/Entity.h"
#include <iostream>
#include <iterator>
#include <map>

namespace gp = gams::platforms;

// Static var. 
int gp::SCRIMMAGEBasePlatform::num_agents = 0;

gams::platforms::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform(
  scrimmage::SimControl& simcontrol,
  madara::knowledge::KnowledgeBase * kb_,
  gams::variables::Sensors * sensors_,
  gams::variables::Self * self_
  ) 
{

  this->knowledge_ = kb_;
  this->self_      = self_;
  this->sensors_   = sensors_;

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::controllers::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform:" \
        " has been initialized. Hijacking SimControl.\n");
        
  if (SCRIMMAGEBasePlatform::num_agents == 0)
  {
      SCRIMMAGEBasePlatform::num_agents = 1;
  } 
  else
  {
      SCRIMMAGEBasePlatform::num_agents++;
  }
  
  madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Spawning entity #%i in SCRIMMAGE..", SCRIMMAGEBasePlatform::num_agents
           );      
           
  knowledge_->set("agent_number", SCRIMMAGEBasePlatform::num_agents);
  
  // TODO spawn and setup entity in simulator
  // There should be N multicontrollers, 1 for each agent.
  auto entities = simcontrol.ents();
  
  // Generate a plane entity as default. Subclass could implement something diff/we could parametrize this later.
  auto ent_params = std::map<std::string, std::string>();
  
  ent_params["team_id"]      = "1";
  ent_params["color"]        = "77 77 225";
  ent_params["count"]        = "1";
  ent_params["health"]       = "1";
  ent_params["radius"]       = "1";
  ent_params["heading"]      = "0";
  ent_params["motion_model"] = "SimpleAircraft";
  ent_params["controller0"]   = "SimpleAircraftControllerPID"; // requires a 0 after the specifier if calling this by code (probably bug in scrimmage's Entity.cpp?
  ent_params["visual_model"] = "zephyr-blue";
  ent_params["autonomy0"]     = "WaypointDispatcher";
  ent_params["autonomy1"]     = "MotorSchemas";
  ent_params["use_variance_all_ents"] = "true";
  ent_params["waypointlist_network"] = "GlobalNetwork";
  ent_params["waypoint_network"]     = "LocalNetwork";
  ent_params["show_shapes"]          = "false";
  ent_params["max_speed"]            = "25";
  ent_params["behaviors"]            = "[ AvoidEntityMS gain='1.0' sphere_of_influence='10' minimum_range='2' ] [ MoveToGoalMS gain='1.0' use_initial_heading='true' goal='-1300,0,100']";
  ent_params["autonomy"] = "MotorSchemas";
  
  // Position offset by number of agents
  auto x_offset = 10 * SCRIMMAGEBasePlatform::num_agents;
  auto y_offset = 0  * SCRIMMAGEBasePlatform::num_agents;
  auto z_offset = 0  * SCRIMMAGEBasePlatform::num_agents;
  
  // Create KRs for 2 reasons: uploading them to KB and utility conversion function of .to_string()
  madara::knowledge::KnowledgeRecord x_offset_kr(x_offset);
  madara::knowledge::KnowledgeRecord y_offset_kr(y_offset);
  madara::knowledge::KnowledgeRecord z_offset_kr(z_offset);
  
  ent_params["x"] = x_offset_kr.to_string();
  ent_params["y"] = y_offset_kr.to_string();
  ent_params["z"] = z_offset_kr.to_string();
  
  // Upload relevant agent info to KB
  knowledge_->set("agent{agent_number}.x_spawn_loc", x_offset_kr);
  knowledge_->set("agent{agent_number}.y_spawn_loc", y_offset_kr);
  knowledge_->set("agent{agent_number}.z_spawn_loc", z_offset_kr);
  
  madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Set Entity <x y z> spawn location to ID #: x: %i y: %i z: %i\n",
           x_offset_kr.to_integer(), y_offset_kr.to_integer(), z_offset_kr.to_integer()
           );
  
  // takes as parameter desc_id, params, hard code to 1 for now
  // generate entity 
  simcontrol.generate_entity(1, ent_params);
  
  madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Created Entity ID #: %i\n",
           SCRIMMAGEBasePlatform::num_agents
           );
         
}


gams::platforms::SCRIMMAGEBasePlatform::~SCRIMMAGEBasePlatform()
{

}

void
gams::platforms::SCRIMMAGEBasePlatform::operator=(const SCRIMMAGEBasePlatform & rhs)
{
  if (this != &rhs)
  {
  
  }
}

int
gams::platforms::SCRIMMAGEBasePlatform::sense(void)
{
   // Read state

   return 1;
}

int
gams::platforms::SCRIMMAGEBasePlatform::analyze(void)
{

   return 0;
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_name(void) const
{

   return std::string("get_name");
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_id(void) const 
{

  return std::string("get_id");
}

double
gams::platforms::SCRIMMAGEBasePlatform::get_accuracy(void) const 
{

  return 1.0;
}

int
gams::platforms::SCRIMMAGEBasePlatform::move(const pose::Position & target, const pose::PositionBounds &bounds)
{

  return 1;
}

const gams::pose::ReferenceFrame & gams::platforms::SCRIMMAGEBasePlatform::get_frame(void) const
{

  gams::pose::ReferenceFrame r;
  return r;
}

gams::platforms::SCRIMMAGEBasePlatformFactory::~SCRIMMAGEBasePlatformFactory()
{

}

gams::platforms::SCRIMMAGEBasePlatformFactory::SCRIMMAGEBasePlatformFactory()
{

}

gams::platforms::BasePlatform *
gams::platforms::SCRIMMAGEBasePlatformFactory::create(
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      variables::Sensors * sensors,
      variables::Platforms * platforms,
      variables::Self * self
)
{
   madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MINOR,
    "entering gams::platforms::SCRIMMAGEBasePlatformFactory::create\n");

  BasePlatform * result(0);

  if (knowledge && sensors && platforms && self)
  {
    if (knowledge->get_num_transports() == 0)
    {
      madara::transport::QoSTransportSettings settings;

      settings.type = madara::transport::MULTICAST;
      settings.hosts.push_back("239.255.0.1:4150");

      knowledge_->attach_transport("", settings);
      knowledge_->activate_transport();

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
         "gams::platforms::SCRIMMAGEBasePlatformFactory::create:" \
        " no transports found, attaching multicast\n");
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::SCRIMMAGEBasePlatformFactory::create:" \
      " creating SCRIMMAGEBasePlatform object\n");

    // specify the model file
    //string model_file;
    //simxUChar is_client_side; // file is on server

//    madara::knowledge::KnowledgeMap::const_iterator client_side_found =
//      args.find("client_side");
//    madara::knowledge::KnowledgeMap::const_iterator model_file_found =
//      args.find("model_file");
//    madara::knowledge::KnowledgeMap::const_iterator resource_dir_found =
//      args.find("resource_dir");

//    if (client_side_found != args.end() &&
//        client_side_found->second.to_integer() == 1)
//    {
//      is_client_side = 1;
//    }
//    else
//    {
//      is_client_side = 0;
//    }

//    if (model_file_found != args.end())
//    {
//      model_file = model_file_found->second.to_string();
//    }
//    else if (resource_dir_found != args.end())
//    {
//      model_file = get_default_model(resource_dir_found->second.to_string());
//    }
//    else
//    {
//      model_file = get_default_model();
//    }

//    result = create_quad(model_file, is_client_side, knowledge, sensors, 
//      platforms, self);
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
       "gams::platforms::VREPQuadFactory::create:" \
      " invalid knowledge, sensors, platforms, or self\n");
  }

  if (result == 0)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::VREPQuadFactory::create:" \
      " error creating VREPQuad object\n");
  }

  return result;
}
