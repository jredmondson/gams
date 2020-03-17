#include "gams/platforms/scrimmage/SCRIMMAGEBasePlatform.h"
#include "scrimmage/entity/Entity.h"
#include <iostream>
#include <iterator>

gams::platforms::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform(
  scrimmage::SimControl& simcontrol) 
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::controllers::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform:" \
        " has been initialized. Hijacking SimControl.\n");
        
  // TODO spawn and setup entity in simulator
  // There should be N multicontrollers, 1 for each agent.
  std::list<scrimmage::EntityPtr> entities = simcontrol.ents();
  
  simcontrol.generate_entity(2);
  
  if (entities.size() <= 0)
  {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "No entities found in simulator");
  } else 
  {
           for (auto const& it : entities) 
          {
              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_ALWAYS,
                "Entity ID: %i",
                it->id());
          }
  }
  
         
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
