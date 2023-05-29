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

#ifndef ROBOCUP_SSL_COMM__MULTICAST_HPP_
#define ROBOCUP_SSL_COMM__MULTICAST_HPP_

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/asio.hpp>  // NOLINT

namespace multicast
{

namespace asio = boost::asio;

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & host, const int port)
  : socket_(io_service_, asio::ip::udp::v4())
  {
    asio::ip::address addr = asio::ip::address::from_string(host);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    socket_.set_option(asio::socket_base::reuse_address(true));
    socket_.set_option(asio::ip::multicast::join_group(addr.to_v4()));
    socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
    socket_.non_blocking(true);
  }

  size_t receive(std::vector<char> & msg)
  {
    boost::system::error_code error;
    const size_t received = socket_.receive(asio::buffer(msg), 0, error);
    if (error && error != asio::error::message_size) {
      throw boost::system::system_error(error);
      return 0;
    }
    return received;
  }

  size_t available() {return socket_.available();}

private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
};

}  // namespace multicast

#endif  // ROBOCUP_SSL_COMM__MULTICAST_HPP_
