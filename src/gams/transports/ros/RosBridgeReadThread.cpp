
#include "gams/loggers/GlobalLogger.h"
#include "RosBridgeReadThread.h"

namespace knowledge = madara::knowledge;


void gams::transports::RosBridgeReadThread::messageCallback (
  const topic_tools::ShapeShifter::ConstPtr& msg,
  const std::string &topic_name )
{
  std::cout << "CALLBACK FROM "  << topic_name << " <" << msg->getDataType() << "> " << msg << std::endl;
  //std::string container = gams::utility::ros::ros_to_gams_name(topic_name);
  //Check if topic is in the topic mapping
  std::map<std::string, std::string>::iterator it = topic_map_.find (topic_name);
  std::string container;

  if (it != topic_map_.end ())
  {
    container = it->second;
  }
  else
  {
    container = gams::utility::ros::ros_to_gams_name (topic_name);
  }

  parser_->parse_message(msg, container);
}

// constructor
gams::transports::RosBridgeReadThread::RosBridgeReadThread (
  const std::string & id,
  const madara::transport::TransportSettings & settings,
  madara::transport::BandwidthMonitor & send_monitor,
  madara::transport::BandwidthMonitor & receive_monitor,
  madara::transport::PacketScheduler & packet_scheduler,
  std::vector<std::string> topics,
  std::map<std::string,std::string> topic_map)
: send_monitor_ (send_monitor),
  receive_monitor_ (receive_monitor),
  packet_scheduler_ (packet_scheduler),
  topics_(topics),
  topic_map_(topic_map)
{
}

// destructor
gams::transports::RosBridgeReadThread::~RosBridgeReadThread ()
{
  //sub_.shutdown();
  delete[] parser_;
}

/**
 * Initialization to a knowledge base.
 **/
void
gams::transports::RosBridgeReadThread::init (knowledge::KnowledgeBase & knowledge)
{
  std::cout << "Initializing RosBridgeReadThread\n" << std::endl;


  // grab the context so we have access to update_from_external  
  context_ = &(knowledge.get_context ());
  
  // setup the receive buffer
  if (settings_.queue_length > 0)
    buffer_ = new char [settings_.queue_length];

  parser_ = new gams::utility::ros::RosParser(&knowledge, "world", "frame1");

  char **argv;
  int argc = 0;
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle node;
  //const char* topic_names[]= { "test1", "odom", "/tf" };
  for (const std::string topic_name: topics_ )
  {
    boost::function<void (const topic_tools::ShapeShifter::ConstPtr&)> callback;
    callback = boost::bind (
      &gams::transports::RosBridgeReadThread::messageCallback, this,
      _1, topic_name );
    subscribers_.push_back (node.subscribe( topic_name, 10, callback));
    std::cout << "subcribed to " << topic_name << std::endl;
  }

}

/**
 * Executes the actual thread logic.
 **/
void
gams::transports::RosBridgeReadThread::run (void)
{
  // prefix for logging purposes
  char * print_prefix = "gams::transports::RosBridgeReadThread";

  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "%s::run: executing\n", print_prefix);

  ros::spinOnce();
  ros::Duration(1).sleep();


  /**
   * this should store the number of bytes read into the buffer after your
   * network, socket, serial port or shared memory read
   **/
  uint32_t bytes_read = 0;

  // header for buffer
  madara::transport::MessageHeader * header = 0;

  /**
   * remote host identifier, often available from socket recvs. This 
   * information can be used with trusted/banned host lists in the
   * process_received_update function later. You would have to fill
   * this in with info from whatever transport you are using for this
   * to be effective.
   **/
  char * remote_host = "";
  
  // will be filled with rebroadcast records after applying filters
  knowledge::KnowledgeMap rebroadcast_records;

  /**
   * Calls filters on buffered data, updates throughput calculations
   **/
  process_received_update (buffer_.get_ptr (), bytes_read, id_, *context_,
    settings_, send_monitor_, receive_monitor_, rebroadcast_records,
    on_data_received_, print_prefix, remote_host, header);
    
  /**
   * The next run of this method will be determined by read_thread_hertz
   * in the QoSTransportSettings class that is passed in.
   **/
  //context_->print(0);
}
