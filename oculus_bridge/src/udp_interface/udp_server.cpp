/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2019, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ----------------------------------------------------------------------------

/**
 * \file      udp_server.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <udp_interface/udp_server.h>

udp_server::udp_server(const std::string &addr, int port) : f_port(port), f_addr(addr)
{
  struct sockaddr_in si_me;

  // create a UDP socket
  if ((f_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    std::cout << "ERROR creating socket" << std::endl;
    exit(1);
  }

  // zero out the structure
  memset((char *)&si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  memset((char *)&si_other, 0, sizeof(si_other));

  // bind socket to port
  if (bind(f_socket, (struct sockaddr *)&si_me, sizeof(si_me)) == -1)
  {
    std::cout << "ERROR binding" << std::endl;
    exit(1);
  }

  std::cout << "Bind to: " << addr << ":" << port << std::endl;
}
udp_server::~udp_server()
{
  close(f_socket);
}

int udp_server::get_socket() const
{
  return f_socket;
}

int udp_server::get_port() const
{
  return f_port;
}

std::string udp_server::get_addr() const
{
  return f_addr;
}

int udp_server::recv(char *msg, size_t max_size)
{
  int s_len = sizeof(si_other);
  return recvfrom(f_socket, msg, max_size, 0, (struct sockaddr *)&si_other, (socklen_t *)(&s_len));
  std::cout << "Client is: " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << std::endl;
}

int udp_server::send(const char *msg, size_t max_size)
{
  return sendto(f_socket, msg, max_size, 0, (struct sockaddr *)&si_other, sizeof(si_other));
}