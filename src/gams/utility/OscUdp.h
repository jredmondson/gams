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


#ifndef _GAMS_UTILITY_OSC_HELPER_H_
#define _GAMS_UTILITY_OSC_HELPER_H_

#include <memory>
#include <map>
#include <vector>
#include <string>

#include <oscpp/client.hpp>
#include <oscpp/server.hpp>
#include <oscpp/print.hpp>

#include "madara/transport/udp/UdpTransport.h"
#include "madara/transport/multicast/MulticastTransport.h"
#include "madara/transport/broadcast/BroadcastTransport.h"
#include "madara/utility/EpochEnforcer.h"

#include "gams/GamsExport.h"

namespace gams
{
  namespace utility
  {
    class GAMS_EXPORT OscUdp
    {
      private:
        /// underlying transport
        std::shared_ptr<madara::transport::UdpTransport> transport_ = 0;

        /// settings 
        madara::transport::QoSTransportSettings settings_;

        /// basically unused
        madara::knowledge::KnowledgeBase kb_;

        /// buffer for sending/receiving
        madara::utility::ScopedArray<char> buffer_ =
          new char[64000];
        
      public:
        typedef  std::map<std::string, std::vector<double>>  OscMap;

        /**
         * Constructor
         **/
        OscUdp() = default;

        /**
         * Processes OSC packets and places them into an OSC map
         * @param packet   the packet to process
         * @param map      a map updated with recent messages
         **/
        size_t pack (void* buffer, size_t size, const OscMap& map)
        {
          OSCPP::Client::Packet packet(buffer, size);
          
          auto bundle =
            packet.openBundle((uint64_t)madara::utility::get_time());
 
          for (auto i : map)
          {
            // open a new message
            auto msg = bundle.openMessage(
              i.first.c_str(), OSCPP::Tags::array(i.second.size()));

            // create the message arguments
            auto args = msg.openArray();
            for (auto arg : i.second)
            {
              args.float32((float)arg);
            }

            // clean up the array and message
            args.closeArray();
            msg.closeMessage();
          }

          // clean up the bundle
          bundle.closeBundle();

          // return the size written
          return packet.size();
        }

        /**
         * Processes OSC packets and places them into an OSC map
         * @param packet   the packet to process
         * @param map      a map updated with recent messages
         **/
        void unpack (const OSCPP::Server::Packet& packet, OscMap & map)
        {
          if (packet.isBundle())
          {
            // Convert to bundle
            OSCPP::Server::Bundle bundle(packet);

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

            // Get argument stream
            OSCPP::Server::ArgStream args(msg.args());

            // Add args to the map
            std::vector<double> double_args;

            while (!args.atEnd())
            {
              double_args.push_back(args.float32());
            }

            if (double_args.size() != 0)
            {
              map[msg.address()] = double_args;
            }
          }
        }

        /**
         * Returns if the UDP-based socket has been created properly
         **/
        inline bool has_socket (void)
        {
          return transport_.get() != 0;
        }

        /**
         * Creates and configures the underlying transport
         * @param config   the configuration of the network. For UDP,
         *                 make sure type is UDP and you have at least
         *                 two hosts setup: [0] local_ip:port, [1] server:port
         **/
        void create_socket (
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

        /**
         * Processes socket receives until empty buffer or max wait time
         * @param values   the values received
         * @param max_wait_seconds the max wait time in seconds (can be <1)
         * @return 0 if success, -1 if bad transport, 1 or 2 for socket issues
         **/
        int receive (OscMap & values, double max_wait_seconds = 0.5)
        {
          int result = -1;
          std::string remote;
          size_t bytes_read;

          if (has_socket())
          {
            madara::utility::EpochEnforcer<madara::utility::Clock> enforcer(
              0.0, max_wait_seconds);

            while (result == 0 || enforcer.is_done())
            {
              result = transport_.get()->receive_buffer(
                buffer_.get(), bytes_read, remote);

              if (result == 0)
              {
                unpack(
                  OSCPP::Server::Packet(buffer_.get(), bytes_read), values);
              }
            }
          }

          return result;
        }

        /**
         * Sends a map of messages and args over the transport 
         * @param values   the values to send
         * @return 0 if success, -1 if bad transport, 1 or 2 for socket issues
         **/
        int send (const OscMap & values)
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

            for (auto address : addresses)
            {
              result = transport_->send_buffer(
                address, buffer_.get(), packed_bytes);
            }
          }

          return result;
        }
    };
  }
}

#endif // _GAMS_UTILITY_OSC_HELPER_H_
