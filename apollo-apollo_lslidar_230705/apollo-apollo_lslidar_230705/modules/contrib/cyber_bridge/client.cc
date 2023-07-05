/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#include "modules/contrib/cyber_bridge/client.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "boost/bind.hpp"

#include "cyber/common/log.h"
#include "cyber/message/protobuf_factory.h"
#include "modules/contrib/cyber_bridge/clients.h"
#include "modules/contrib/cyber_bridge/node.h"

enum {
  OP_REGISTER_DESC = 1,
  OP_ADD_READER = 2,
  OP_ADD_WRITER = 3,
  OP_PUBLISH = 4,
};

Client::Client(Node* node, Clients* clients, boost::asio::ip::tcp::socket s)
    : node(*node), clients(*clients), socket(std::move(s)) {
  auto endpoint = socket.remote_endpoint();
  AINFO << "Client [" << endpoint.address() << ":" << endpoint.port()
        << "] connected";
}

Client::~Client() {}

void Client::start() {
  socket.async_read_some(
      boost::asio::buffer(temp, sizeof(temp)),
      boost::bind(&Client::handle_read, shared_from_this(),
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void Client::stop() { socket.close(); }

void Client::handle_read(const boost::system::error_code& ec,
                         std::size_t size) {
  if (!ec) {
    ADEBUG << "Received " << size << " bytes";
    buffer.insert(buffer.end(), temp, temp + size);

    size_t size = buffer.size();

    while (size >= sizeof(uint8_t)) {
      if (buffer[0] == OP_REGISTER_DESC) {
        handle_register_desc();
      } else if (buffer[0] == OP_ADD_READER) {
        handle_add_reader();
      } else if (buffer[0] == OP_ADD_WRITER) {
        handle_add_writer();
      } else if (buffer[0] == OP_PUBLISH) {
        handle_publish();
      } else {
        AERROR << "Unknown operation received from client ("
               << uint32_t(buffer[0]) << "), disconnecting client";
        clients.stop(shared_from_this());
        return;
      }

      if (size == buffer.size()) {
        break;
      }
      size = buffer.size();
    }

    start();
    return;
  }

  if (ec == boost::asio::error::eof) {
    // remote side closed connection
    clients.stop(shared_from_this());
    node.remove(shared_from_this());
    return;
  }

  if (ec != boost::asio::error::operation_aborted) {
    AERROR << "Client read failed, disconnecting" << ec;
    clients.stop(shared_from_this());
    node.remove(shared_from_this());
    return;
  }
}

void Client::handle_write(const boost::system::error_code& ec) {
  if (ec) {
    if (ec != boost::asio::error::operation_aborted) {
      AERROR << "Client write failed, disconnecting" << ec;
      clients.stop(shared_from_this());
      node.remove(shared_from_this());
    }
    return;
  }

  std::lock_guard<std::mutex> lock(publish_mutex);
  writing.clear();
  if (!pending.empty()) {
    writing.swap(pending);
    boost::asio::async_write(
        socket, boost::asio::buffer(writing.data(), writing.size()),
        boost::bind(&Client::handle_write, shared_from_this(),
                    boost::asio::placeholders::error));
  }
}

// [1] [count] [string] ... [string]
void Client::handle_register_desc() {
  if (sizeof(uint8_t) + sizeof(uint32_t) > buffer.size()) {
    ADEBUG << "handle_register_desc too short";
    return;
  }

  uint32_t count = get32le(1);

  std::vector<std::string> desc;

  bool complete = true;
  size_t offset = sizeof(uint8_t) + sizeof(uint32_t);
  for (uint32_t i = 0; i < count; i++) {
    if (offset + sizeof(uint32_t) > buffer.size()) {
      ADEBUG << "handle_register_desc too short";
      complete = false;
      break;
    }

    uint32_t size = get32le(offset);
    offset += sizeof(uint32_t);

    if (offset + size > buffer.size()) {
      ADEBUG << "handle_register_desc too short";
      complete = false;
      break;
    }

    desc.push_back(std::string(reinterpret_cast<char*>(&buffer[offset]), size));
    offset += size;
  }

  if (complete) {
    ADEBUG << "OP_REGISTER_DESC, count = " << count;

    auto factory = apollo::cyber::message::ProtobufFactory::Instance();
    for (const auto& s : desc) {
      factory->RegisterPythonMessage(s);
    }

    buffer.erase(buffer.begin(), buffer.begin() + offset);
  }
}

// [2] [channel] [type]
void Client::handle_add_reader() {
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size()) {
    ADEBUG << "handle_add_reader too short header";
    return;
  }

  size_t offset = sizeof(uint8_t);

  uint32_t channel_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + channel_length > buffer.size()) {
    ADEBUG << "handle_add_reader short1 " << offset + channel_length << " "
           << buffer.size();
    return;
  }

  std::string channel(reinterpret_cast<char*>(&buffer[offset]), channel_length);
  offset += channel_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size()) {
    ADEBUG << "handle_add_reader short2 " << offset + type_length << " "
           << buffer.size();
    return;
  }

  std::string type(reinterpret_cast<char*>(&buffer[offset]), type_length);
  offset += type_length;

  ADEBUG << "OP_NEW_READER, channel = " << channel << ", type = " << type;

  node.add_reader(channel, type, shared_from_this());

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

// [3] [channel] [type]
void Client::handle_add_writer() {
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size()) {
    ADEBUG << "handle_new_writer too short header";
    return;
  }

  size_t offset = sizeof(uint8_t);

  uint32_t channel_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + channel_length > buffer.size()) {
    ADEBUG << "handle_new_writer short1 " << offset + channel_length << " "
           << buffer.size();
    return;
  }

  std::string channel(reinterpret_cast<char*>(&buffer[offset]), channel_length);
  offset += channel_length;

  uint32_t type_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + type_length > buffer.size()) {
    ADEBUG << "handle_new_writer short2 " << offset + type_length << " "
           << buffer.size();
    return;
  }

  std::string type(reinterpret_cast<char*>(&buffer[offset]), type_length);
  offset += type_length;

  ADEBUG << "OP_NEW_WRITER, channel = " << channel << ", type = " << type;

  node.add_writer(channel, type, shared_from_this());

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

// [4] [channel] [message]
void Client::handle_publish() {
  if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size()) {
    return;
  }

  size_t offset = sizeof(uint8_t);

  uint32_t channel_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + channel_length > buffer.size()) {
    return;
  }

  std::string channel(reinterpret_cast<char*>(&buffer[offset]), channel_length);
  offset += channel_length;

  uint32_t message_length = get32le(offset);
  offset += sizeof(uint32_t);
  if (offset + message_length > buffer.size()) {
    return;
  }

  std::string message(reinterpret_cast<char*>(&buffer[offset]), message_length);
  offset += message_length;

  ADEBUG << "OP_PUBLISH, channel = " << channel;

  node.publish(channel, message);

  buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void fill_data(std::vector<uint8_t>* data, const std::string& channel,
               const std::string& msg) {
  data->reserve(data->size() + sizeof(uint8_t) + sizeof(uint32_t) +
                channel.size() + sizeof(uint32_t) + msg.size());

  data->push_back(OP_PUBLISH);

  data->push_back(uint8_t(channel.size() >> 0));
  data->push_back(uint8_t(channel.size() >> 8));
  data->push_back(uint8_t(channel.size() >> 16));
  data->push_back(uint8_t(channel.size() >> 24));
  const uint8_t* channel_data =
      reinterpret_cast<const uint8_t*>(channel.data());
  data->insert(data->end(), channel_data, channel_data + channel.size());

  data->push_back(uint8_t(msg.size() >> 0));
  data->push_back(uint8_t(msg.size() >> 8));
  data->push_back(uint8_t(msg.size() >> 16));
  data->push_back(uint8_t(msg.size() >> 24));
  const uint8_t* msg_data = reinterpret_cast<const uint8_t*>(msg.data());
  data->insert(data->end(), msg_data, msg_data + msg.size());
}

void Client::publish(const std::string& channel, const std::string& msg) {
  std::lock_guard<std::mutex> lock(publish_mutex);
  if (writing.empty()) {
    fill_data(&writing, channel, msg);
    boost::asio::async_write(
        socket, boost::asio::buffer(writing.data(), writing.size()),
        boost::bind(&Client::handle_write, shared_from_this(),
                    boost::asio::placeholders::error));
  } else if (pending.size() < MAX_PENDING_SIZE) {
    fill_data(&pending, channel, msg);
  } else {
    // If pending size is larger than MAX_PENDING_SIZE, discard the message.
    AERROR << "Pending size too large. Discard message.";
  }
}

uint32_t Client::get32le(size_t offset) const {
  return buffer[offset + 0] | (buffer[offset + 1] << 8) |
         (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
}
