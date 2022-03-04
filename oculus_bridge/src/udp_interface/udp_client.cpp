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
 * \file      udp_client.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <udp_interface/udp_client.h>

udp_client::udp_client(const std::string &address, int port) : f_port(port), f_address(address)
{
  if ((f_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    std::cout << "ERROR creating socket" << std::endl;
    exit(1);
  }

  memset((char *)&si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(port);

  if (inet_aton(address.c_str(), &si_other.sin_addr) == 0)
  {
    std::cout << "ERROR inet_aton" << std::endl;
    exit(1);
  }

  std::cout << "Server is: " << address << ":" << port << std::endl;
}

int udp_client::get_socket() const
{
  return f_socket;
}

int udp_client::get_port() const
{
  return f_port;
}

std::string udp_client::get_addr() const
{
  return f_address;
}

int udp_client::recv(char *msg, size_t size)
{
  int s_len = sizeof(si_other);
  return recvfrom(f_socket, msg, size, 0, (struct sockaddr *)&si_other, (socklen_t *)(&s_len));
}

int udp_client::send(const char *msg, size_t size)
{
  return sendto(f_socket, msg, size, 0, (struct sockaddr *)&si_other, sizeof(si_other));
}

udp_client::~udp_client()
{
  close(f_socket);
}