#include "gams/platforms/scrimmage/SCRIMMAGEBasePlatform.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/State.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/motion/Controller.h"
#include "scrimmage/autonomy/Autonomy.h"

#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <gams/plugins/scrimmage/GAMSAutonomy.h>

#include "Eigen/Dense"
#include <iostream>
#include <iterator>
#include <map>

namespace gp = gams::platforms;

// Static var. 
int gp::SCRIMMAGEBasePlatform::num_agents = 1;

gams::platforms::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform(
  scrimmage::SimControl * simcontrol_,
  madara::knowledge::KnowledgeBase * kb_,
  gams::variables::Sensors * sensors_,
  gams::variables::Self * self_
  ) : self_id(0), gams::platforms::BasePlatform(kb_, sensors_, self_)
{

  this->knowledge_ = kb_;
  this->self_      = self_;
  this->sensors_   = sensors_;
  this->simcontrol = simcontrol_;
  
  this->knowledge_->print();
  
  this->self_id = this->knowledge_->get(".id");
  
  // SCRIMMAGE will start its IDs at 1, so we adjust based on that (inc all ID by 1)
  int temp = this->self_id.to_integer();
  temp = temp + 1;
  this->self_id.set_value(temp);

  if (this->knowledge_ && this->sensors_)
  {
      threader_.set_data_plane(*kb_);
      
      gams::variables::Sensors::iterator it = sensors_->find("coverage");
      if (it == sensors_->end ()) // create coverage sensor
      {
        // get origin
        gams::pose::Position origin (gams::pose::gps_frame());
        madara::knowledge::containers::NativeDoubleArray origin_container;
        origin_container.set_name ("sensor.coverage.origin", *knowledge_, 3);
        origin.from_container (origin_container);

        // establish sensor
        gams::variables::Sensor* coverage_sensor =
          new gams::variables::Sensor ("coverage", knowledge_, 2.5, origin);
        (*sensors_)["coverage"] = coverage_sensor;
      }
      
    (*sensors_)["coverage"] = (*sensors_)["coverage"];
    status_.init_vars (*knowledge_, get_id ());
    
    /**
    * the following should be set when movement is available in your
    * platform. If on construction, movement should be possible, then
    * feel free to keep this uncommented. Otherwise, set it somewhere else
    * in analyze or somewhere else when appropriate to enable movement.
    * If you never enable movement_available, movement based algorithms are
    * unlikely to ever move with your platform.
    **/
    status_.movement_available = 1;
  
  }
  
  this->spawn_entity();         
}

/*
  TODO Spawns an entity in the simulator with configurable parameters
  
  Note. This requires that MAX_NUM_ENTITIES in MissionParse.cpp is > than the number of    agents allowed to spawn. There is a bug where if <count>0</count> in the XML file, it will segfault. I will open a pull request with the SCRIMMAGE maintainers. Using their API other than this will cause a segfault. The SCRIMMAGE SimControl API is MOSTLY unusable other than this method used in this function to spawn agents.
*/
void
gams::platforms::SCRIMMAGEBasePlatform::spawn_entity()
{
     madara_logger_ptr_log(
     gams::loggers::global_logger.get(),
     gams::loggers::LOG_ALWAYS,
     "gams::controllers::SCRIMMAGEBasePlatform::spawn_entity()" \
     " Attempting to generate entity in SCRIMMAGE\n"
     );
     
     // Get initial pose and spawn entity at that pose.
     madara::knowledge::containers::NativeDoubleVector init_pos(".initial_pose", *this->knowledge_);
     
     scrimmage::State s;
     
     s.pos() << init_pos[0], init_pos[1], init_pos[2];

     s.quat() = scrimmage::Quaternion(0, 0, 0);
     
     scrimmage::PublisherPtr p = this->simcontrol->plugin()->advertise("GlobalNetwork", "GenerateEntity");
     auto msg = std::make_shared<scrimmage::Message<scrimmage_msgs::GenerateEntity>>();
     
     auto kv_id = msg->data.add_entity_param();
     std::string id = this->self_id.to_string();
     kv_id->set_key("id");
     kv_id->set_value(id);
     
     scrimmage::set(msg->data.mutable_state(), s);
     msg->data.set_entity_tag("gen_straight");
     
     madara_logger_ptr_log(
     gams::loggers::global_logger.get(),
     gams::loggers::LOG_ALWAYS,
     "gams::controllers::SCRIMMAGEBasePlatform::spawn_entity()" \
     " Spawning agent %d at origin(x: %f, y: %f, z: %f)\n",
     this->self_id,
     s.pos()[0], 
     s.pos()[1], 
     s.pos()[2]
     );
     
     p->publish(msg);
}

scrimmage::EntityPtr
gams::platforms::SCRIMMAGEBasePlatform::get_entity()
{
   for (auto ent : simcontrol->ents())
   {
       if (ent->id().id() == this->self_id.to_integer())
       {
          madara_logger_ptr_log(
          gams::loggers::global_logger.get(),
          gams::loggers::LOG_ALWAYS,
          "Entity found #: %i\n",
          this->self_id.to_integer()
          );
          
          return ent;
       }
   }
   
   madara_logger_ptr_log(
   gams::loggers::global_logger.get(),
   gams::loggers::LOG_ALWAYS,
   "Entity not found in simulation scene yet.\n"
   );

   return NULL;
}


gams::platforms::SCRIMMAGEBasePlatform::~SCRIMMAGEBasePlatform()
{
    threader_.terminate();
    threader_.wait();
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
   // Read state. This appears never to work. The EntityPtr in the map is ALWAYS null ??
   std::shared_ptr<std::unordered_map<int, scrimmage::EntityPtr>> ent_map;
   ent_map = simcontrol->id_to_entity_map();
   
   madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Sensing vars for #: %i\n",
           this->self_id.to_integer()
           );
   
   if (ent_map)
   {

      // this isnt able to find the ID of the agent in sim.  
      scrimmage::EntityPtr this_ent     = (*ent_map)[this->self_id.to_integer()];
      
      if (this_ent)
      {
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Entity available in simulation\n"
           );
           
           scrimmage::StatePtr this_state    = (*this_ent).state();
           // Read current entity state
           Eigen::Vector3d this_pos          = (*this_state).pos();
           Eigen::Vector3d this_vel          = (*this_state).vel();
           Eigen::Vector3d this_ang_vel      = (*this_state).ang_vel();
           scrimmage::Quaternion this_orient = (*this_state).quat(); 
           
           // Location inside Self/Agent/.location. Store there.
           this->self_->agent.location.set(0, this_pos.x());
           this->self_->agent.location.set(1, this_pos.y());
           this->self_->agent.location.set(2, this_pos.z());

           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Set position\n"
           );

           // Store angular velocity
           this->self_->agent.velocity.set(0, this_vel.x());
           this->self_->agent.velocity.set(1, this_vel.y());
           this->self_->agent.velocity.set(2, this_vel.z());

           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Set Angular Velocity\n"
           );

           // Store orientation
           // r = 0 p = 1 y = 2
           this->self_->agent.orientation.set(0, this_orient.roll());
           this->self_->agent.orientation.set(1, this_orient.pitch());
           this->self_->agent.orientation.set(2, this_orient.yaw());
           
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Set Orientation\n"
           );
         
      } else
      {
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Entity not created in simulation yet.\n"
           );
      }
//      
//      // No option to set angular velocity in GAMS? Not sure TODO
//      
//      // What else do the base algorithms in GAMS need? What do the SimpleControllers need? Looks like it requires only desired state, and state as set by autonomy to operate on.
//      
   }

   return gams::platforms::PLATFORM_OK;
}

int
gams::platforms::SCRIMMAGEBasePlatform::analyze(void)
{
   return gams::platforms::PLATFORM_OK;
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_name(void) const
{
   return "SCRIMMAGEBasePlatform";
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_id(void) const 
{
   return "SCRIMMAGEBasePlatform";
}

double
gams::platforms::SCRIMMAGEBasePlatform::get_accuracy(void) const 
{
  return 0.2;
}

int
gams::platforms::SCRIMMAGEBasePlatform::move(const gams::pose::Position & target, const pose::PositionBounds &bounds)
{
   madara_logger_ptr_log(
   gams::loggers::global_logger.get(),
   gams::loggers::LOG_ALWAYS,
   "Moving robot...\n"
   );
   
  int result = gams::platforms::PLATFORM_MOVING;

  // Get entity
  // Scrimmage starts its ID at 1
  scrimmage::EntityPtr ent = (*this->simcontrol->id_to_entity_map())[this->self_id.to_integer()+1];
  
  if (ent)
  {
        scrimmage::StatePtr current_state = ent->state();
        
        gams::pose::Pose now(current_state->pos()[0],
                             current_state->pos()[1],
                             current_state->pos()[2]);
  
        madara_logger_ptr_log(
        gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "Moving robot to %s\n", target.to_string().c_str()
        );
        
        gams::pose::Pose p(target);
        
        double accuracy = now.distance_to(p);
        
        if (accuracy <= this->get_accuracy())
        {
            result = gams::platforms::PLATFORM_ARRIVED;
            madara_logger_ptr_log(
            gams::loggers::global_logger.get(),
            gams::loggers::LOG_ALWAYS,
            "Platform arrived to goal"
            );
            
            // Set desired state to current state as we have satisfied conditions
            
            for (auto autonomy : ent->autonomies())
            {
               autonomy->set_desired_state(current_state);
            }
            
        } else
        {
        
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Distance from goal: %f\n", accuracy
           );
       
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Moving robot to x: %f y: %f: z: %f\n", p.x(), p.y(), p.z()
           );
           
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Orienting robot to r: %f p: %f: y: %f\n", p.rx(), p.ry(), p.rz()
           );
      
           // Set the entities desired state and its state
           // Can I access the desired state from here?
           scrimmage::StatePtr des_state = std::make_shared<scrimmage::State>();
           
           double x_ = p.x();
           double y_ = p.y();
           double z_ = p.z();
           
           double w_ = 0;
           
           Eigen::Vector3d xyz(x_,y_,z_);
           
           des_state->set_pos(xyz);
           
           // Until we get an autonomy that generates this for us
           des_state->set_quat(current_state->quat());
           
           for (auto autonomy : ent->autonomies())
           {
               autonomy->set_desired_state(des_state);
           }
           
           madara_logger_ptr_log(
           gams::loggers::global_logger.get(),
           gams::loggers::LOG_ALWAYS,
           "Set Desired State to x: %f y: %f z: %f \n",
           p.x(), p.y(), p.z()
           );
       }
  } else
  {
       madara_logger_ptr_log(
       gams::loggers::global_logger.get(),
       gams::loggers::LOG_ALWAYS,
       "Entity null.\n"
       );
  } 
  
  return result;
}

const gams::pose::ReferenceFrame & gams::platforms::SCRIMMAGEBasePlatform::get_frame(void) const
{
  return gams::pose::default_frame();
}

//gams::variables::PlatformStatus *
//gams::platforms::SCRIMMAGEBasePlatform::get_platform_status(void)
//{
//    return &this->status_;
//}


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
  return new SCRIMMAGEBasePlatform(new scrimmage::SimControl(), knowledge, sensors, self);
}
