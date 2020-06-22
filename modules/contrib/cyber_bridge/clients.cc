/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#include "modules/contrib/cyber_bridge/clients.h"

#include <memory>

#include "modules/contrib/cyber_bridge/client.h"

Clients::Clients() {}

Clients::~Clients() {}

void Clients::start(std::shared_ptr<Client> client) {
  clients.insert(client);
  client->start();
}

void Clients::stop(std::shared_ptr<Client> client) {
  clients.erase(client);
  client->stop();
}

void Clients::stop_all() {
  for (auto& client : clients) {
    client->stop();
  }
  clients.clear();
}
