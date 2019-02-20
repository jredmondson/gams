/**
 * Copyright (c) 2019 James Edmondson. All Rights Reserved.
 *
 **/

/**
 * @file OscUdp.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a helper class for sending/receiving Open
 * Stage Control messages
 **/

#include "OscUdp.h"

size_t
gams::utility::OscUdp::pack (void* buffer, size_t size, const OscMap& map)
{
  OSCPP::Client::Packet packet(buffer, size);
  
  OSCPP::Client::Packet * cur_p = &
    packet.openBundle((uint64_t)madara::utility::get_time());

  for (auto i : map)
  {
    // open a new message
    cur_p->openMessage(
      i.first.c_str(), OSCPP::Tags::array(i.second.size()));

    // create the message arguments
    // cur_p->openArray();

    for (auto arg : i.second)
    {
      cur_p->float32((float)arg);
    }

    // clean up the array and message
    // cur_p->closeArray();
    cur_p->closeMessage();
  }

  // clean up the bundle
  cur_p->closeBundle();

  // return the size written
  return packet.size();
}

/**
 * Processes OSC packets and places them into an OSC map
 * @param packet   the packet to process
 * @param map      a map updated with recent messages
 **/
void
gams::utility::OscUdp::unpack (const OSCPP::Server::Packet& packet, OscMap & map)
{
  packet.size();

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::utility::OscUdp::unpack: " \
    " unpacking %zu bytes. Checking for bundle\n",
    packet.size());

  if (packet.isBundle())
  {
    // Convert to bundle
    OSCPP::Server::Bundle bundle(packet);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::utility::OscUdp::unpack: " \
      " found bundle\n");

    // Get packet stream
    OSCPP::Server::PacketStream packets(bundle.packets());

    while (!packets.atEnd())
    {
      unpack(packets.next(), map);
    }
  }
  else
  {
    // Convert to message
    OSCPP::Server::Message msg(packet);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::utility::OscUdp::unpack: " \
      " found message %s with %zu args\n", msg.address(), msg.args().size());

    // Get argument stream
    OSCPP::Server::ArgStream args(msg.args());

    // // Get argument stream
    // OSCPP::Server::ArgStream params(args.array());

    // Add args to the map
    std::vector<double> double_args;

    while (!args.atEnd())
    {
      double_args.push_back(args.float32());
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
        "gams::utility::OscUdp::unpack: " \
        " %s temp double_args now at size %zu\n",
        msg.address(), double_args.size());
    }

    if (double_args.size() != 0)
    {
      map[msg.address()] = double_args;
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
        "gams::utility::OscUdp::unpack: " \
        " adding %s with %zu values to map\n",
        msg.address(), double_args.size());
    }
  }
  
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::utility::OscUdp::unpack: " \
    " returning with %zu values\n",
    map.size());

}

/**
 * Creates and configures the underlying transport
 * @param config   the configuration of the network. For UDP,
 *                 make sure type is UDP and you have at least
 *                 two hosts setup: [0] local_ip:port, [1] server:port
 **/
void
gams::utility::OscUdp::create_socket (
  madara::transport::QoSTransportSettings& settings)
{
  settings_ = settings;

  // disable read thread settings
  settings_.no_receiving = true;

  // we're not implementing fragmentation right now
  settings_.queue_length = 64000;

  if (settings.type == madara::transport::UDP)
  {
    transport_ =
      std::make_shared<madara::transport::UdpTransport>(
        "", kb_.get_context(), settings_, true);
  }
  else if (settings.type == madara::transport::MULTICAST)
  {
    transport_ =
      std::make_shared<madara::transport::MulticastTransport>(
        "", kb_.get_context(), settings_, true);
  }
  else if (settings.type == madara::transport::BROADCAST)
  {
    transport_ =
      std::make_shared<madara::transport::BroadcastTransport>(
        "", kb_.get_context(), settings_, true);
  }
}

int
gams::utility::OscUdp::receive (OscMap & values, double max_wait_seconds)
{
  int result = -1;
  boost::asio::ip::udp::endpoint remote;
  size_t bytes_read;

  if (has_socket())
  {
    madara::utility::EpochEnforcer<madara::utility::Clock> enforcer(
      0.0, max_wait_seconds);

    do
    {
      result = transport_.get()->receive_buffer(
        buffer_.get(), bytes_read, remote);

      if (result == 0 && bytes_read != 0)
      {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::utility::OscUdp::receive: " \
          " received %zu bytes. Calling unpack\n",
          bytes_read);

        unpack(
          OSCPP::Server::Packet(buffer_.get(), bytes_read), values);
      }
    } while (result == 0 && !enforcer.is_done());
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::utility::OscUdp::receive: " \
    " returning result %d with %zu values\n",
    result, values.size());

  return result;
}

int
gams::utility::OscUdp::send (const OscMap & values)
{
  int result = 0;
  std::string remote;
  size_t max_send = 64000;
  size_t packed_bytes;

  if (has_socket())
  {
    const std::vector<boost::asio::ip::udp::endpoint> & addresses =
      transport_->get_udp_endpoints();

    packed_bytes = pack(buffer_.get(), max_send, values);

    for (size_t i = 1; i < addresses.size(); ++i)
    {
      result = transport_->send_buffer(
        addresses[i], buffer_.get(), packed_bytes);
    }
  }

  return result;
}
