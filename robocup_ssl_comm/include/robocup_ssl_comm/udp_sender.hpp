// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOCUP_SSL_COMM__UDP_SENDER_HPP_
#define ROBOCUP_SSL_COMM__UDP_SENDER_HPP_

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/asio.hpp>  // NOLINT

namespace udp_sender
{

namespace asio = boost::asio;

class UDPSender
{
public:
  UDPSender(const std::string & host, const int port)
  : socket_(io_service_, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0))
  {
    asio::ip::udp::resolver resolver(io_service_);
    asio::ip::udp::resolver::query query(host, std::to_string(port));
    endpoint_ = *resolver.resolve(query);
  }

  void send(const std::string & str)
  {
    socket_.send_to(asio::buffer(str), endpoint_);
  }

private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint endpoint_;
};

}  // namespace udp_sender

#endif  // ROBOCUP_SSL_COMM__UDP_SENDER_HPP_
