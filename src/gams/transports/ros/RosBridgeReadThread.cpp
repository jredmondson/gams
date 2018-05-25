
#include "gams/loggers/GlobalLogger.h"
#include "RosBridgeReadThread.h"

namespace knowledge = madara::knowledge;

// constructor
gams::transports::RosBridgeReadThread::RosBridgeReadThread (
  const std::string & id,
  const madara::transport::TransportSettings & settings,
  madara::transport::BandwidthMonitor & send_monitor,
  madara::transport::BandwidthMonitor & receive_monitor,
  madara::transport::PacketScheduler & packet_scheduler)
: send_monitor_ (send_monitor),
  receive_monitor_ (receive_monitor),
  packet_scheduler_ (packet_scheduler)
{
}

// destructor
gams::transports::RosBridgeReadThread::~RosBridgeReadThread ()
{
}

/**
 * Initialization to a knowledge base.
 **/
void
gams::transports::RosBridgeReadThread::init (knowledge::KnowledgeBase & knowledge)
{
  // grab the context so we have access to update_from_external  
  context_ = &(knowledge.get_context ());
  
  // setup the receive buffer
  if (settings_.queue_length > 0)
    buffer_ = new char [settings_.queue_length];
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
}
