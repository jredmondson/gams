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
  size_t result = 0;

  for (auto i : map)
  {
    // @Alex Rozgo. Here is where the OSC pack magic should happen
    // map is a STL map of string->vector<double>. We just have to
    // pack according to the types. I've left scaffolding in here for
    // checking the message addresses by their ending. Feel free to
    // do what you feel is necessary

    // velocity changes are arrays of floats
    if (
      madara::utility::ends_with (i.first, "/velocity/xy") ||
      madara::utility::ends_with (i.first, "/velocity/z") ||
      madara::utility::ends_with (i.first, "/yaw")
    )
    {

    }
    // rotation and position are streams of floats
    else if (
      madara::utility::ends_with (i.first, "/pos") ||
      madara::utility::ends_with (i.first, "/rot"))
    {

    }
  }

  // return the size written
  return result;
}


void
gams::utility::OscUdp::unpack (void* buffer, size_t size, OscMap & map)
{
  // @Alex Rozgo. I've removed all of this and changed the function signature
  // to be more generic
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

        unpack(buffer_.get(), bytes_read, values);
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
