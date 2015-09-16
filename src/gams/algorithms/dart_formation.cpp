#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <assert.h>
#include <math.h>

#include "madara/knowledge_engine/Knowledge_Base.h"
#include "madara/knowledge_engine/Knowledge_Record.h"
#include "madara/knowledge_engine/Functions.h"
#include "madara/knowledge_engine/containers/Integer_Vector.h"
#include "madara/knowledge_engine/containers/Double_Vector.h"
#include "madara/transport/Packet_Scheduler.h"
#include "madara/threads/Threader.h"
#include "madara/filters/Generic_Filters.h"

#define _GAMS_VREP_ 1
#include "gams/controllers/Base_Controller.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/platforms/vrep/VREP_Base.h"
#include "gams/variables/Self.h"
#include "gams/utility/GPS_Position.h"
#include "gams/utility/Axes.h"

#include "dmpl/Reference.hpp"
#include "dmpl/ArrayReference.hpp"
#include <stdlib.h>
#include "madara/utility/Utility.h"

#include "dart_formation.h"

// begin dmpl namespace
namespace dmpl
{
// typedefs
typedef   Madara::Knowledge_Record::Integer   Integer;

// namespace shortcuts
namespace engine = Madara::Knowledge_Engine;
namespace threads = Madara::Threads;

namespace containers = engine::Containers;

namespace controllers = gams::controllers;

namespace platforms = gams::platforms;

namespace variables = gams::variables;

using containers::Reference;

using containers::ArrayReference;

using Madara::knowledge_cast;


engine::Knowledge_Base knowledge;

// Needed as a workaround for non-const-correctness in Madara; use carefully
inline engine::Function_Arguments &__strip_const(const engine::Function_Arguments &c)
{
  return const_cast<engine::Function_Arguments &>(c);
}

inline engine::Function_Arguments &__chain_set(engine::Function_Arguments &c, int i, Madara::Knowledge_Record v)
{
  c[i] = v;
  return c;
}

// default transport variables
std::string host ("");
std::vector<std::string> platform_params;
std::string platform_name ("debug");
typedef void (*PlatformInitFn)(const std::vector<std::string> &, engine::Knowledge_Base &);
typedef std::map<std::string, PlatformInitFn> PlatformInitFns;
PlatformInitFns platform_init_fns;
const std::string default_multicast ("239.255.0.1:4150");
Madara::Transport::QoS_Transport_Settings settings;
int write_fd (-1);
ofstream expect_file;

// Containers for commonly used variables
// Global variables
Reference<unsigned int> id(knowledge, ".id");
Reference<unsigned int>  num_processes(knowledge, ".num_processes");
ArrayReference<unsigned int, 5> mbarrier_COLLISION_AVOIDANCE(knowledge, "mbarrier_COLLISION_AVOIDANCE");
double max_barrier_time (-1);
engine::Knowledge_Update_Settings private_update (true);

// number of participating processes
unsigned int processes (5);

// Defining program-specific constants
double BottomY = -2.25;
#define INITS 0
double LeftX = -2.25;
#define MOVE 4
#define NEXT 1
#define REQUEST 2
double RightX = 2.25;
double TopY = 2.25;
#define WAITING 3
unsigned int X = 10;
unsigned int Y = 10;

// Defining program-specific global variables
ArrayReference<_Bool, 5> init(knowledge, "init");
_Bool var_init_init (0);

ArrayReference<_Bool, 5, 10, 10> lock(knowledge, "lock");

ArrayReference<short, 5> lx(knowledge, "lx");
short var_init_lx (0);

ArrayReference<short, 5> ly(knowledge, "ly");
short var_init_ly (0);

ArrayReference<_Bool, 5> step(knowledge, "step");
_Bool var_init_step (0);


// Defining program-specific local variables
Reference<short> state(knowledge, ".state");
short var_init_state (0);

Reference<_Bool> waypointValid(knowledge, ".waypointValid");
_Bool var_init_waypointValid (0);

Reference<short> x(knowledge, ".x");
short var_init_x (0);

Reference<short> xp(knowledge, ".xp");
short var_init_xp (0);

Reference<short> xt(knowledge, ".xt");
short var_init_xt (0);

Reference<short> y(knowledge, ".y");
short var_init_y (0);

Reference<short> yp(knowledge, ".yp");
short var_init_yp (0);

Reference<short> yt(knowledge, ".yt");
short var_init_yt (0);


// target (GNU_CPP) specific thunk

#define GNU_WIN

/*
int GRID_MOVE(unsigned char xp,unsigned char yp,double z)
{
  return rand() < (RAND_MAX / 10 * 6);
}
*/

int my_sleep (int seconds)
{
  Madara::Utility::sleep (seconds);
  return 0;
}

int round = 0;
int xi,yi;

void print_int(int i)
{
  printf("%i\n", i);
}

void print_line(int _X)
{
  printf("-");
  for(int i = 0;i < _X;++i) printf("--");
  printf("\n");
}

void print_state(int st, int _X,int _Y,int id, int x, int y, int xf, int yf)
{
  if(round == 0) {
    xi = x; yi = y;
  }

  for(int i = 0;i < 150;++i) printf("\n");
  
  printf("round = %d : state = %d : id = %d\n", ++round, st, id);
  printf("_X = %d _Y = %d\n",_X,_Y);
  print_line(_X);
  for(int i = 0;i < _Y;++i) {
    printf("|");
    for(int j = 0;j < _X;++j) {
      //printf("i = %d j = %d\n", i, j);
      if(j == xf && i == yf) printf("o|");        
      else if(j == x && i == y) printf("%d|",id);
      else printf(" |");
    }
    printf("\n");
    print_line(_X);
  }
}



template < class ContainerT >
void tokenize(const std::string& str, ContainerT& tokens,
              const std::string& delimiters = " ", bool trimEmpty = false)
{
   std::string::size_type pos, lastPos = 0;

   typedef typename ContainerT::value_type value_type;
   typedef typename ContainerT::size_type size_type;

   while(true)
   {
      pos = str.find_first_of(delimiters, lastPos);
      if(pos == std::string::npos)
      {
         pos = str.length();

         if(pos != lastPos || !trimEmpty)
            tokens.push_back(value_type(str.data()+lastPos,
                  (size_type)pos-lastPos ));

         break;
      }
      else
      {
         if(pos != lastPos || !trimEmpty)
            tokens.push_back(value_type(str.data()+lastPos,
                  (size_type)pos-lastPos ));
      }

      lastPos = pos + 1;
   }
}

// Forward declaring global functions


// Forward declaring node functions

Madara::Knowledge_Record
uav_COLLISION_AVOIDANCE (engine::Function_Arguments & args, engine::Variables & vars);
Madara::Knowledge_Record
uav_NEXT_XY (engine::Function_Arguments & args, engine::Variables & vars);
Madara::Knowledge_Record
uav_StartingPosition (engine::Function_Arguments & args, engine::Variables & vars);
Madara::Knowledge_Record
uav_WAYPOINT (engine::Function_Arguments & args, engine::Variables & vars);

gams::platforms::Base_Platform *platform = NULL;
int grid_x = 0, grid_y = 0;
double grid_leftX = NAN, grid_rightX = NAN, grid_topY = NAN, grid_bottomY = NAN, grid_cellX = NAN, grid_cellY = NAN;
void GRID_INIT(int x, int y, double leftX, double rightX, double topY, double bottomY)
{
  grid_x = x;
  grid_y = y;
  grid_leftX = leftX;
  grid_rightX = rightX;
  grid_topY = topY;
  grid_bottomY = bottomY;
  grid_cellX = (grid_rightX - grid_leftX) / (grid_x-1);
  grid_cellY = (grid_bottomY - grid_topY) / (grid_y-1);
}

void GRID_PLACE(int x, int y, double alt = 0.0)
{
  knowledge.set(".initial_x", grid_leftX + x * grid_cellX);
  knowledge.set(".initial_y", grid_topY + y * grid_cellY);
  knowledge.set(".initial_alt", alt);
}

int GRID_MOVE(int x, int y, double alt = NAN, double epsilon = 0.1)
{
  int ret = platform->move(gams::utility::Position(grid_leftX + x * grid_cellX, grid_topY + y * grid_cellY, alt), epsilon);
  return ret != 2;
}

double GET_X()
{
  return 0;
}

double GET_Y()
{
  return 0;
}

double GET_LAT()
{
  if(platform == NULL) return NAN;
  gams::utility::Position *pos = platform->get_position();
  double lat = pos->x;
  delete pos;
  return lat;
}

double GET_LNG()
{
  if(platform == NULL) return NAN;
  gams::utility::Position *pos = platform->get_position();
  double lng = pos->y;
  delete pos;
  return lng;
}

int ROTATE(double angle)
{
  std::cout << "Rotate: " << angle << std::endl;
  return platform->rotate(gams::utility::Axes(0, 0, angle));
}


Madara::Knowledge_Record
REMODIFY_BARRIERS (engine::Function_Arguments &,
  engine::Variables & vars)
{
  mbarrier_COLLISION_AVOIDANCE[id].mark_modified();

  return Integer (0);
}

Madara::Knowledge_Record
REMODIFY_GLOBALS (engine::Function_Arguments & args,
  engine::Variables & vars)
{
  // Remodifying common global variables
  REMODIFY_BARRIERS(args, vars);
  // Remodifying program-specific global variables
  init[id].mark_modified();
  lock[id].mark_modified();
  lx[id].mark_modified();
  ly[id].mark_modified();
  step[id].mark_modified();
  return Integer (0);
}

// Defining global functions

// Defining node functions

// @BarrierSync
// @Criticality 4
// @Period 200000
// @PlatformController
// @WCExecTimeNominal 15000
// @WCExecTimeOverload 30000
Madara::Knowledge_Record
uav_COLLISION_AVOIDANCE (engine::Function_Arguments & args, engine::Variables & vars)
{
  // Declare local variables

  {
    (void) (print_state (state, X, Y, id, x, y, xt, yt));
  }
  if ((state == INITS))
  {
lock[id][x][y] = Integer (1);
    if ((id == Integer (0)))
    {
lx[id] = x;
ly[id] = y;
    }
init[id] = Integer (1);
    state = NEXT;
  }
  if ((state == NEXT))
  {
    if ((id == Integer (0)))
    {
      if (((x == xt) && (y == yt)))
      {
        return Integer(0);
      }
      if ((id == 0 && ((step[1] != step[id]) || (step[2] != step[id]) || (step[3] != step[id]) || (step[4] != step[id]))) || 
        (id == 1 && ((step[2] != step[id]) || (step[3] != step[id]) || (step[4] != step[id]))) || 
        (id == 2 && ((step[3] != step[id]) || (step[4] != step[id]))) || 
        (id == 3 && ((step[4] != step[id]))))
      {
        return Integer(0);
      }
    }
    else
    {
      if (((x == xt) && (y == yt)))
      {
step[id] = step[Integer (0)];
        return Integer(0);
      }
    }
    if ((uav_NEXT_XY (
           __strip_const(engine::Function_Arguments(0))
          , vars).to_integer() == 0 ? false : true))
    {
      return Integer(0);
    }
    if ((id == Integer (0)))
    {
step[id] = (!step[id]);
    }
    state = REQUEST;
  }
  else
  {
    if ((state == REQUEST))
    {
      if ((id == 1 && ((lock[0][xp][yp] != Integer (0)))) || 
        (id == 2 && ((lock[0][xp][yp] != Integer (0)) || (lock[1][xp][yp] != Integer (0)))) || 
        (id == 3 && ((lock[0][xp][yp] != Integer (0)) || (lock[1][xp][yp] != Integer (0)) || (lock[2][xp][yp] != Integer (0)))) || 
        (id == 4 && ((lock[0][xp][yp] != Integer (0)) || (lock[1][xp][yp] != Integer (0)) || (lock[2][xp][yp] != Integer (0)) || (lock[3][xp][yp] != Integer (0)))))
      {
        return Integer(0);
      }
lock[id][xp][yp] = Integer (1);
      state = WAITING;
    }
    else
    {
      if ((state == WAITING))
      {
        if ((id == 0 && ((lock[1][xp][yp] != Integer (0)) || (lock[2][xp][yp] != Integer (0)) || (lock[3][xp][yp] != Integer (0)) || (lock[4][xp][yp] != Integer (0)))) || 
          (id == 1 && ((lock[2][xp][yp] != Integer (0)) || (lock[3][xp][yp] != Integer (0)) || (lock[4][xp][yp] != Integer (0)))) || 
          (id == 2 && ((lock[3][xp][yp] != Integer (0)) || (lock[4][xp][yp] != Integer (0)))) || 
          (id == 3 && ((lock[4][xp][yp] != Integer (0)))))
        {
          return Integer(0);
        }
        state = MOVE;
      }
      else
      {
        if ((state == MOVE))
        {
          if ((GRID_MOVE (xp, yp, 0.5)))
          {
            return Integer(0);
          }
lock[id][x][y] = Integer (0);
          x = xp;
          y = yp;
          state = NEXT;
        }
      }
    }
  }

  // Insert return statement, in case user program did not
  return Integer(0);
}

Madara::Knowledge_Record
uav_NEXT_XY (engine::Function_Arguments & args, engine::Variables & vars)
{
  // Declare local variables

  if ((!waypointValid))
  {
    // retType->type == 273
    return (Integer(Integer (1)));
  }
  xp = x;
  yp = y;
  if ((x < xt))
  {
    xp = (x + Integer (1));
  }
  else
  {
    if ((x > xt))
    {
      xp = (x - Integer (1));
    }
    else
    {
      if ((y < yt))
      {
        yp = (y + Integer (1));
      }
      else
      {
        yp = (y - Integer (1));
      }
    }
  }
  // retType->type == 273
  return (Integer(Integer (0)));

  // Insert return statement, in case user program did not
  return Integer(0);
}

// @InitSim
Madara::Knowledge_Record
uav_StartingPosition (engine::Function_Arguments & args, engine::Variables & vars)
{
  // Declare local variables

  {
    (void) (GRID_INIT (X, Y, LeftX, RightX, TopY, BottomY));
  }
  {
    (void) (GRID_PLACE (x, y, 0.5));
  }

  // Insert return statement, in case user program did not
  return Integer(0);
}

// @Criticality 3
// @Period 200000
// @WCExecTimeNominal 10000
// @WCExecTimeOverload 20000
Madara::Knowledge_Record
uav_WAYPOINT (engine::Function_Arguments & args, engine::Variables & vars)
{
  // Declare local variables

  if (((id != Integer (0)) && (init[Integer (0)] == Integer (0))))
  {
    waypointValid = Integer (0);
    return Integer(0);
  }
  else
  {
    waypointValid = Integer (1);
  }
  if ((id == Integer (0)))
  {
ly[id] = yp;
lx[id] = xp;
  }
  if ((id == Integer (1)))
  {
    xt = (lx[Integer (0)] + Integer (1));
    yt = (ly[Integer (0)] + Integer (1));
  }
  else
  {
    if ((id == Integer (2)))
    {
      xt = (lx[Integer (0)] - Integer (1));
      yt = (ly[Integer (0)] + Integer (1));
    }
    else
    {
      if ((id == Integer (3)))
      {
        xt = (lx[Integer (0)] - Integer (1));
        yt = (ly[Integer (0)] - Integer (1));
      }
      else
      {
        if ((id == Integer (4)))
        {
          xt = (lx[Integer (0)] + Integer (1));
          yt = (ly[Integer (0)] - Integer (1));
        }
      }
    }
  }

  // Insert return statement, in case user program did not
  return Integer(0);
}


class Algo : public gams::algorithms::Base_Algorithm, protected threads::Base_Thread
{
public:
  Algo (
    unsigned period,
    const std::string &exec_func,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
    const std::string &platform_name = "",
    variables::Sensors * sensors = 0,
    variables::Self * self = 0);
  ~Algo (void);
  virtual int analyze (void);
  virtual int plan (void);
  virtual int execute (void);
  virtual void init (engine::Knowledge_Base & context);
  virtual void init_platform ();
  virtual void run (void);
  virtual void cleanup (void);
  virtual void start (threads::Threader &threader);
protected:
  unsigned _period; //-- period in us
  controllers::Base_Controller loop;
  engine::Knowledge_Base *knowledge_;
  std::string _exec_func, _platform_name;
};

class SyncAlgo : public Algo
{
public:
  SyncAlgo (
    unsigned period,
    const std::string &exec_func,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
    const std::string &platform_name = "",
    variables::Sensors * sensors = 0,
    variables::Self * self = 0);
  ~SyncAlgo (void);
  virtual int analyze (void);
  virtual int plan (void);
  virtual int execute (void);
  virtual void init (engine::Knowledge_Base & context);
  virtual void run (void);
  virtual void cleanup (void);
protected:
  int phase;
  std::string mbarrier;
  Madara::Knowledge_Engine::Wait_Settings wait_settings;
  engine::Compiled_Expression round_logic;
  std::map <std::string, bool>  barrier_send_list;
  std::stringstream barrier_string, barrier_data_string, barrier_sync;
  engine::Compiled_Expression barrier_logic;
  engine::Compiled_Expression barrier_data_logic;
  engine::Compiled_Expression barrier_sync_logic;
};

Algo::Algo (
    unsigned period,
    const std::string &exec_func,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge,
    const std::string &platform_name,
    variables::Sensors * sensors,
    variables::Self * self) : loop(*knowledge), _platform_name(platform_name),
      Base_Algorithm (knowledge, 0, sensors, self), knowledge_(knowledge),
            _period(period), _exec_func(exec_func)
{
}

Algo::~Algo (void)
{
}

int Algo::analyze (void)
{
  return 0;
}

int Algo::plan (void)
{
  return 0;
}

void Algo::init (engine::Knowledge_Base & context)
{
  loop.init_vars (settings.id, processes);
  if(_platform_name != "") init_platform ();
  loop.init_algorithm (this);
}

void Algo::run (void)
{
  loop.run_once(); 
}

void Algo::init_platform ()
{
  loop.init_platform (_platform_name);
  platform = loop.get_platform();
}

void Algo::cleanup (void)
{
}

void Algo::start (threads::Threader &threader)
{
  std::cout << "Starting thread: " << _exec_func << " at period " << _period << " us" << std::endl;
  double hertz = 1000000.0 / _period;
  threader.run(hertz, _exec_func, this);
}

int Algo::execute (void)
{
  knowledge_->evaluate (_exec_func + "()");
  return 0;
}

SyncAlgo::SyncAlgo (
    unsigned period,
    const std::string &exec_func,
    Madara::Knowledge_Engine::Knowledge_Base * knowledge,
    const std::string &platform_name,
    variables::Sensors * sensors,
    variables::Self * self) : phase(0), mbarrier("mbarrier_" + exec_func),
      Algo (period, exec_func, knowledge, platform_name, sensors, self)
{
  wait_settings.max_wait_time = 0;
  wait_settings.poll_frequency = .1;

  round_logic = knowledge_->compile (
    knowledge_->expand_statement (_exec_func + " (); ++" + mbarrier + ".{.id}"));
}

SyncAlgo::~SyncAlgo (void)
{
}

void SyncAlgo::init (engine::Knowledge_Base & context)
{
  bool started = false;

  barrier_send_list [knowledge_->expand_statement ("" + mbarrier + ".{.id}")] = true;

  barrier_string << "REMODIFY_BARRIERS () ;> ";
  barrier_data_string << "REMODIFY_GLOBALS () ;> ";
  barrier_sync << "" + mbarrier + ".";
  barrier_sync << settings.id;
  barrier_sync << " = (" + mbarrier + ".";
  barrier_sync << settings.id;

  // create barrier check for all lower ids
  for (unsigned int i = 0; i < settings.id; ++i)
  {
    if (started)
    {
      barrier_string << " && ";
      barrier_data_string << " && ";
    }

    barrier_string << "" + mbarrier + ".";
    barrier_string << i;
    barrier_string << " >= " + mbarrier + ".";
    barrier_string << settings.id;
    barrier_data_string << "" + mbarrier + ".";
    barrier_data_string << i;
    barrier_data_string << " >= " + mbarrier + ".";
    barrier_data_string << settings.id;
    barrier_sync << " ; ";
    barrier_sync << "" + mbarrier + ".";
    barrier_sync << i;

    if (!started)
      started = true;
  }

  // create barrier check for all higher ids
  for (int64_t i = settings.id + 1; i < processes; ++i)
  {
    if (started)
    {
      barrier_string << " && ";
      barrier_data_string << " && ";
    }

    barrier_string << "" + mbarrier + ".";
    barrier_string << i;
    barrier_string << " >= " + mbarrier + ".";
    barrier_string << settings.id;
    barrier_data_string << "" + mbarrier + ".";
    barrier_data_string << i;
    barrier_data_string << " >= " + mbarrier + ".";
    barrier_data_string << settings.id;
    barrier_sync << " ; ";
    barrier_sync << "" + mbarrier + ".";
    barrier_sync << i;

    if (!started)
      started = true;
  }

  barrier_sync << ")";

  // Compile frequently used expressions
  std::cout << "barrier_string: " << barrier_string.str() << std::endl;
  barrier_logic = knowledge_->compile (barrier_string.str ());
  barrier_data_logic = knowledge_->compile (barrier_data_string.str ());
  barrier_sync_logic = knowledge_->compile (barrier_sync.str ());
  Algo::init(context);
}

void SyncAlgo::run (void)
{
  Algo::run(); 
}

void SyncAlgo::cleanup (void)
{
}

int SyncAlgo::analyze (void)
{
  return 0;
}

int SyncAlgo::plan (void)
{
  return 0;
}

int SyncAlgo::execute (void)
{
  {
    // Pre-round barrier increment
    if(phase == 0)
    {
      wait_settings.send_list = barrier_send_list; 
      wait_settings.delay_sending_modifieds = true; 
      knowledge_->evaluate ("++" + mbarrier + ".{.id}", wait_settings); 
      phase++;
    }
    if(phase == 1)
    {
      // remodify our globals and send all updates 
      wait_settings.send_list.clear (); 
      wait_settings.delay_sending_modifieds = false; 
      // first barrier for new data from previous round 
      if(knowledge_->evaluate (barrier_data_logic, wait_settings).to_integer()) 
        phase++;
    }
    if(phase == 2)
    {
      // Send only barrier information 
      wait_settings.send_list = barrier_send_list; 
      // Execute main user logic 
      wait_settings.delay_sending_modifieds = true; 
      knowledge_->evaluate (round_logic, wait_settings);
      phase++;
    }
    if(phase == 3)
    {
      // second barrier for waiting on others to finish round 
      // Increment barrier and only send barrier update 
      wait_settings.send_list = barrier_send_list; 
      wait_settings.delay_sending_modifieds = false; 
      if(knowledge_->evaluate (barrier_logic, wait_settings).to_integer()) 
        phase = 0;
    }
  }
  return 0;
}

using namespace dmpl;
template<class T> std::string to_string(const T &in)
{
  std::stringstream ss;
  ss << in;
  return ss.str();
}

void init_vrep(const std::vector<std::string> &params, engine::Knowledge_Base &knowledge)
{
  if(params.size() >= 2 && params[1].size() > 0)
    knowledge.set(".vrep_host", params[1]);
  else
    knowledge.set(".vrep_host", "127.0.0.1");
  if(params.size() >= 3 && params[2].size() > 0)
    knowledge.set(".vrep_port", params[2]);
  else
    knowledge.set(".vrep_port", to_string(19905+settings.id));
  if(params.size() >= 4 && params[3].size() > 0)
    knowledge.set(".vrep_sw_position", params[3]);
  else
    knowledge.set(".vrep_sw_position", "40.4464255,-79.9499426");
  if(params.size() >= 5 && params[4].size() > 0)
    knowledge.set(".vrep_uav_move_speed", params[4]);
  else
    knowledge.set(".vrep_uav_move_speed", "0.4");
  knowledge.set("vrep_ready", "1");
}

void dart_formation_init ()
{
  static bool did_init = false;
  if(!did_init)
  {
    settings.type = Madara::Transport::MULTICAST;
    platform_init_fns["vrep"] = init_vrep;
    platform_init_fns["vrep-uav"] = init_vrep;
    platform_init_fns["vrep-heli"] = init_vrep;
    platform_init_fns["vrep-ant"] = init_vrep;
    platform_init_fns["vrep-uav-ranger"] = init_vrep;

    if (settings.hosts.size () == 0)
    {
      // setup default transport as multicast
      settings.hosts.push_back (default_multicast);
      settings.add_receive_filter (Madara::Filters::log_aggregate);
      settings.add_send_filter (Madara::Filters::log_aggregate);
    }

    settings.queue_length = 1000000;

    settings.set_deadline(1);

    // configure the knowledge base with the transport settings
    knowledge.attach_transport(host, settings);

    // NODE: uav
    // @MadaraCriticality 1
    // @MadaraPeriod 100001
    // @MadaraWCExecTimeNominal 10000
    // @MadaraWCExecTimeOverload 20000
    // Binding common variables

    // Binding program-specific global variables
    init[settings.id] = var_init_init;


    lx[settings.id] = var_init_lx;

    ly[settings.id] = var_init_ly;

    step[settings.id] = var_init_step;


    // Binding program-specific local variables
    state = var_init_state;

    waypointValid = var_init_waypointValid;

    x = var_init_x;

    xp = var_init_xp;

    xt = var_init_xt;

    y = var_init_y;

    yp = var_init_yp;

    yt = var_init_yt;


    // Defining common functions

    knowledge.define_function ("REMODIFY_BARRIERS", REMODIFY_BARRIERS);

    knowledge.define_function ("REMODIFY_GLOBALS", REMODIFY_GLOBALS);

    // Defining global functions for MADARA


    // Defining node functions for MADARA

    knowledge.define_function ("COLLISION_AVOIDANCE", uav_COLLISION_AVOIDANCE);
    knowledge.define_function ("NEXT_XY", uav_NEXT_XY);
    knowledge.define_function ("StartingPosition", uav_StartingPosition);
    knowledge.define_function ("WAYPOINT", uav_WAYPOINT);

    // Initialize commonly used local variables
    id = settings.id;
    num_processes = processes;
    if(id < 0 || id >= processes) {
      std::cerr << "Invalid node id: " << settings.id << "  valid range: [0, " << processes - 1 << "]" << std::endl;
      exit(1);
    }
    PlatformInitFns::iterator init_fn = platform_init_fns.find(platform_name);
    if(init_fn != platform_init_fns.end())
      init_fn->second(platform_params, knowledge);
    knowledge.evaluate("StartingPosition()");
    knowledge.set("begin_sim", "1");
    did_init = true;
  }
}

::gams::algorithms::Base_Algorithm * Algo_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge_,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  dart_formation_init();
  return new Algo(10000, args[0].to_string(), &knowledge);
}

::gams::algorithms::Base_Algorithm * SyncAlgo_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge_,
  platforms::Base_Platform * platform_,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  platform = platform_;
  dart_formation_init();
  return new SyncAlgo(10000, args[0].to_string(), &knowledge);
}

} // end dmpl namespace
