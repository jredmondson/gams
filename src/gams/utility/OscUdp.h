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

#include "madara/transport/udp/UdpTransport.h"
#include "madara/transport/multicast/MulticastTransport.h"
#include "madara/transport/broadcast/BroadcastTransport.h"
#include "madara/utility/EpochEnforcer.h"

#include "gams/GamsExport.h"
#include "gams/loggers/GlobalLogger.h"

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

        ~OscUdp()
        {
          // force transport to clear first then kb should clean
          transport_ = 0;
        }

        /**
         * Processes OSC packets and places them into an OSC map
         * @param buffer   the buffer to pack into
         * @param size     the size of the buffer
         * @param map      a map updated with recent messages
         **/
        size_t pack (void* buffer, size_t size, const OscMap& map);

        /**
         * Processes OSC packets and places them into an OSC map
         * @param buffer   the buffer to unpack from
         * @param size     the size of the buffer
         * @param map      a map updated with recent messages
         **/
        void unpack (void* buffer, size_t size, OscMap & map);

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
          madara::transport::QoSTransportSettings& settings);

        /**
         * Processes socket receives until empty buffer or max wait time
         * @param values   the values received
         * @param max_wait_seconds the max wait time in seconds (can be <1)
         * @return 0 if success, -1 if bad transport, 1 or 2 for socket issues
         **/
        int receive (OscMap & values, double max_wait_seconds = 0.5);

        /**
         * Sends a map of messages and args over the transport 
         * @param values   the values to send
         * @return 0 if success, -1 if bad transport, 1 or 2 for socket issues
         **/
        int send (const OscMap & values);
    };
  }
}

#endif // _GAMS_UTILITY_OSC_HELPER_H_
