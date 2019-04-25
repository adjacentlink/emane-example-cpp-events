/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Description:
//
// example application sending and receiving an EMANE pathloss event
//  using an ipv4 or ipv6 event service channel

#include <iostream>
#include <thread>
#include <netdb.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <uuid.h>
#include <sys/eventfd.h>
#include <signal.h>

#include "event.pb.h"
#include "pathlossevent.pb.h"

namespace
{
  // surrogate application configuration
  const char * MULTICAST_ADDRESS{"224.1.2.8"};
  const char * MULTICAST_SERVICE{"45703"};
  const char * MULTICAST_DEVICE{"lo"};
  const bool MULTICAST_LOOP_ENABLE{true};
  const int MULTICAST_LOOP_TTL{32};

  // eventfd object used to signal thread to exit
  int iSignalEvent{};

  void sighandler(int)
  {
    eventfd_write(iSignalEvent,1);
  }
}

// thread procedure used to receive events and print info
void eventProcessor(int iSocket,int iSignal)
{
  char buf[65535];
  fd_set rfds;
  int iMaxFd{std::max(iSocket,iSignal)};
  while(1)
    {
      FD_ZERO(&rfds);
      FD_SET(iSocket,&rfds);
      FD_SET(iSignal,&rfds);

      if(select(iMaxFd+1, &rfds,nullptr,nullptr,nullptr) > 0)
        {
          if(FD_ISSET(iSocket,&rfds))
            {
              ssize_t len = recv(iSocket,buf,sizeof(buf),0);

              // verify that length is large enough to hold the length prefix
              //  framing
              if(len >= 2)
                {
                  // use to avoid dereferencing type-punned pointer will
                  //  break strict-aliasing rules warning
                  std::uint16_t * pu16LengthPrefixFraming{reinterpret_cast<std::uint16_t *>(buf)};

                  std::uint16_t u16MessageSize{ntohs(*pu16LengthPrefixFraming)};

                  // properly framed message. This might seem unnecessary for
                  //  a datagram but it allows a consistent format if the
                  //  transport is switched to a stream in the future.
                  if(u16MessageSize == len - 2)
                    {
                      EMANEMessage::Event msg{};

                      // de-serialize the event message
                      if(msg.ParseFromArray(&buf[2],u16MessageSize))
                        {
                          // a single event message may include multple event payloads
                          for(const auto & serialization : msg.data().serializations())
                            {
                              // determine which event type to handle using the event id
                              switch(serialization.eventid())
                                {
                                case 100:
                                  std::cout<<"received location event for nem "
                                           <<serialization.nemid()
                                           <<std::endl
                                           <<" use EMANEMessage::LocationEvent to decode..."
                                           <<std::endl;
                                  break;

                                case 101:
                                  {
                                    // full example parsing a pathloss event
                                    std::cout<<"received pathloss event for nem "
                                             <<serialization.nemid()
                                             <<std::endl;

                                    EMANEMessage::PathlossEvent event{};

                                    if(event.ParseFromString(serialization.data()))
                                      {
                                        for(const auto & pathloss : event.pathlosses())
                                          {
                                            std::cout<<" nem "
                                                     <<pathloss.nemid()
                                                     <<" forward pathloss dB "
                                                     <<pathloss.forwardpathlossdb()
                                                     <<" reverse pathloss dB "
                                                     <<pathloss.reversepathlossdb()
                                                     <<std::endl;
                                          }
                                      }
                                  }
                                  break;

                                case 102:
                                  std::cout<<"received antenna profile event for nem "
                                           <<serialization.nemid()
                                           <<std::endl
                                           <<" use EMANEMessage::AntennaProfileEvent to decode..."
                                           <<std::endl;
                                  break;

                                case 103:
                                  std::cout<<"received comm effect event for nem "
                                           <<serialization.nemid()
                                           <<std::endl
                                           <<" use EMANEMessage::CommEffectEvent to decode..."
                                           <<std::endl;
                                  break;
                                default:
                                  std::cout<<"received unknown event "
                                           <<serialization.eventid()
                                           <<" for nem "
                                           <<serialization.nemid()
                                           <<std::endl;
                                }
                            }
                        }
                      else
                        {
                          std::cerr<<"received a malformed event message"<<std::endl;
                        }

                    }
                  else
                    {
                      std::cerr<<"received an invalid length prefix framed message"<<std::endl;
                    }
                }
            }

          // eventfd object
          if(FD_ISSET(iSignal,&rfds))
            {
              return;
            }
        }
    }
}

int main(int, char *[])
{
  addrinfo * pMulticastAddrInfo{};

  // set up addrinfo hints
  addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  int iRet{};

  // lookup the ipv4 or ipv6 address info
  if((iRet = getaddrinfo(MULTICAST_ADDRESS,
                         MULTICAST_SERVICE,
                         &hints,
                         &pMulticastAddrInfo)) != 0)
    {
      std::cerr<<gai_strerror(iRet)<<std::endl;
      return EXIT_FAILURE;
    }

  int iSock{};

  // create the mutlicast scoket
  if((iSock = socket(pMulticastAddrInfo->ai_family,
                     pMulticastAddrInfo->ai_socktype,
                     0)) == -1)
    {
      perror("socket");
      return EXIT_FAILURE;
    }

  // set ipv4 or ipv6 options
  if(pMulticastAddrInfo->ai_family == AF_INET)
    {
      std::uint8_t u8Option{};

      if(MULTICAST_LOOP_TTL)
        {
          u8Option = MULTICAST_LOOP_TTL;

          if(setsockopt(iSock,
                        IPPROTO_IP,IP_MULTICAST_TTL,
                        (void*)&u8Option,
                        sizeof(u8Option)) < 0)
            {
              perror("setsockopt IP_MULTICAST_TTL");
              return EXIT_FAILURE;

            }
        }

      if(MULTICAST_LOOP_ENABLE)
        {
          u8Option = 1;

          // set the multicast loopback
          if(setsockopt(iSock,IPPROTO_IP,IP_MULTICAST_LOOP,
                        &u8Option,
                        sizeof(u8Option)) < 0)
            {
              perror("setsockopt IP_MULTICAST_LOOP:");
              return EXIT_FAILURE;
            }
        }

      // multicast group info used for join
      ip_mreq mreq;

      memset(&mreq,0,sizeof(mreq));

      memcpy(&mreq.imr_multiaddr,
             &reinterpret_cast<sockaddr_in*>(pMulticastAddrInfo->ai_addr)->sin_addr,
             sizeof(mreq.imr_multiaddr));

      if(MULTICAST_DEVICE)
        {
          ifreq ifr;
          memset(&ifr,0,sizeof(ifr));
          strncpy(ifr.ifr_name,MULTICAST_DEVICE,IFNAMSIZ);

          // get the ip address
          if(ioctl(iSock, SIOCGIFADDR, &ifr) < 0)
            {
              perror("ioctl SIOCGIFADDR");
              return EXIT_FAILURE;
            }

          // set the multicast interface
          if(setsockopt(iSock,
                        IPPROTO_IP,
                        IP_MULTICAST_IF,
                        &reinterpret_cast<sockaddr_in*>(&ifr.ifr_addr)->sin_addr.s_addr,
                        sizeof(in_addr)) < 0)
            {
              perror("setsockopt IP_MULTICAST_IF");
              return EXIT_FAILURE;
            }

          mreq.imr_interface.s_addr = reinterpret_cast<sockaddr_in*>(&ifr.ifr_addr)->sin_addr.s_addr;
        }
      else
        {
          mreq.imr_interface.s_addr = INADDR_ANY;
        }

      if(setsockopt(iSock,
                    IPPROTO_IP,
                    IP_ADD_MEMBERSHIP,
                    reinterpret_cast<void*>(&mreq),
                    sizeof(mreq)) < 0)
        {
          perror("setsockopt IP_ADD_MEMBERSHIP");
          return EXIT_FAILURE;
        }
    }

  else if(pMulticastAddrInfo->ai_family == AF_INET6)
    {
      int iOption{};

      if(MULTICAST_LOOP_TTL)
        {
          iOption = MULTICAST_LOOP_TTL;

          if(setsockopt(iSock,
                        IPPROTO_IPV6,
                        IPV6_MULTICAST_HOPS,
                        &iOption,
                        sizeof(iOption)) < 0)
            {
              perror("setsockopt IPV6_MULTICAST_HOPS");
              return EXIT_FAILURE;
            }
        }

      if(MULTICAST_LOOP_ENABLE)
        {
          iOption = 1;

          if(setsockopt(iSock,
                        IPPROTO_IPV6,
                        IPV6_MULTICAST_LOOP,
                        &iOption,
                        sizeof(iOption)) < 0)
            {
              perror("setsockopt IPV6_MULTICAST_LOOP");
              return EXIT_FAILURE;
            }
        }

      ipv6_mreq mreq6;

      memset(&mreq6,0,sizeof(mreq6));

      memcpy(&mreq6.ipv6mr_multiaddr,
             &reinterpret_cast<sockaddr_in6 *>(pMulticastAddrInfo->ai_addr)->sin6_addr,
             sizeof(mreq6.ipv6mr_multiaddr));

      mreq6.ipv6mr_interface = 0;

      if(MULTICAST_DEVICE)
        {
          unsigned int iIndex{if_nametoindex(MULTICAST_DEVICE)};

          if(setsockopt(iSock,
                        IPPROTO_IPV6,
                        IPV6_MULTICAST_IF,
                        &iIndex,
                        sizeof(iIndex)) < 0)
            {
              perror("setsockopt IPV6_MULTICAST_IF");
              return EXIT_FAILURE;
            }

          mreq6.ipv6mr_interface = iIndex;
        }

      if(setsockopt(iSock,
                    IPPROTO_IPV6,
                    IPV6_ADD_MEMBERSHIP,
                    reinterpret_cast<void *>(&mreq6),
                    sizeof(mreq6)) < 0)
        {
          perror("setsockopt IPV6_ADD_MEMBERSHIP");
          return EXIT_FAILURE;
        }
    }
  else
    {
      std::cerr<<"unknown address family"<<std::endl;
      return EXIT_FAILURE;
    }

  int iOption{1};

  if(setsockopt(iSock,
                SOL_SOCKET,
                SO_REUSEADDR,
                reinterpret_cast<void*>(&iOption),
                sizeof(iOption)) < 0)
    {
      perror("setsockopt  SO_REUSEADDR");
      return EXIT_FAILURE;
    }

  if(bind(iSock,pMulticastAddrInfo->ai_addr,pMulticastAddrInfo->ai_addrlen) < 0)
    {
      perror("bind");
      return EXIT_FAILURE;
    }

  if((iSignalEvent = eventfd(0,0)) == -1)
    {
      perror("eventfd");
      return EXIT_FAILURE;
    }

  // example terminates using 'Ctl-c' or 'Ctl-\'
  struct sigaction action;

  memset(&action,0,sizeof(action));

  action.sa_handler = sighandler;

  sigaction(SIGINT,&action,nullptr);

  sigaction(SIGQUIT,&action,nullptr);

  // start the receive thread
  std::thread t1(eventProcessor,iSock,iSignalEvent);

  // all emane event publishers need a UUID
  uuid_t uuid;
  uuid_generate(uuid);

  // all published events have an incremental sequence number
  std::uint64_t u64EventPublisherSequence{0};

  // sample pathloss event for 10 nodes with illustrative values
  EMANEMessage::PathlossEvent pathlossEvent;

  for(int i = 1; i <= 10; ++i)
    {
      auto pPathlossMessage = pathlossEvent.add_pathlosses();

      pPathlossMessage->set_nemid(i);

      pPathlossMessage->set_forwardpathlossdb(90+i);

      pPathlossMessage->set_reversepathlossdb(90+i);
    }

  // serialize the pathloss event - it will be treated as
  //  opaque data by the emulator framework
  std::string sSerialization{};

  pathlossEvent.SerializeToString(&sSerialization);

  // create the event message
  EMANEMessage::Event event;

  auto pData = event.mutable_data();

  auto pSerialization = pData->add_serializations();

  // set the NEM destination - use 0 for all NEMs
  pSerialization->set_nemid(0);

  // set the event id of the payload - in this case
  //  Pathloss event
  // From emane/events/eventids.h:
  //  #define EMANE_EVENT_LOCATION 100
  //  #define EMANE_EVENT_PATHLOSS 101
  //  #define EMANE_EVENT_ANTENNA_PROFILE 102
  //  #define EMANE_EVENT_COMMEFFECT 103
  pSerialization->set_eventid(101);

  // set the opaque payload (pathloss event data)
  pSerialization->set_data(sSerialization);

  // set the event publisher UUID
  event.set_uuid(reinterpret_cast<const char *>(uuid),sizeof(uuid));

  // set the event publisher sequence number
  event.set_sequencenumber(++u64EventPublisherSequence);

  // serialize the event message
  event.SerializeToString(&sSerialization);

  // event messages are sent using length prefix framing - the
  //  multicast datagram starts with the length of the serialized
  //  event message as a 16-bit value in network byte order followed
  //  by the serialized event message
  //
  std::uint16_t u16EventSize{htons(static_cast<uint16_t>(sSerialization.size()))};

  msghdr msg;
  iovec msgvec[2];

  memset(&msg,0,sizeof(msg));

  msg.msg_iov = msgvec;
  msg.msg_iovlen = 2;

  msg.msg_name = pMulticastAddrInfo->ai_addr;
  msg.msg_namelen = pMulticastAddrInfo->ai_addrlen;

  // length prefix
  msgvec[0].iov_base = &u16EventSize;
  msgvec[0].iov_len = sizeof(u16EventSize);

  // serialized message
  msgvec[1].iov_base = const_cast<char *>(sSerialization.c_str());
  msgvec[1].iov_len = sSerialization.size();

  if(sendmsg(iSock,&msg,0) < 0)
    {
      perror("sendmsg");
      return EXIT_FAILURE;
    }

  // wait for the receive thread to complete
  t1.join();

  std::cout<<"bye"<<std::endl;

  return EXIT_SUCCESS;
}
