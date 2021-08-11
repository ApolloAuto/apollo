/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#pragma once

#include <memory>
#include <unordered_set>

#include "boost/asio.hpp"

class Client;

class Clients {
 public:
  Clients();
  ~Clients();

  void start(std::shared_ptr<Client> client);
  void stop(std::shared_ptr<Client> client);
  void stop_all();

 private:
  std::unordered_set<std::shared_ptr<Client>> clients;
};
