/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace apollo {
namespace cyber {
class Node;
template <class T>
class Writer;
template <class T>
class Reader;

namespace message {
class PyMessageWrap;
}
}  // namespace cyber
}  // namespace apollo

class Client;

class Node {
 public:
  Node();
  ~Node();

  void remove(std::shared_ptr<Client> client);

  void add_reader(const std::string& channel, const std::string& type,
                  std::shared_ptr<Client> client);
  void add_writer(const std::string& channel, const std::string& type,
                  std::shared_ptr<Client> client);

  void publish(const std::string& channel, const std::string& data);

 private:
  std::unique_ptr<apollo::cyber::Node> node;
  std::mutex mutex;

  struct Writer {
    std::string desc;
    std::string type;
    std::shared_ptr<
        apollo::cyber::Writer<apollo::cyber::message::PyMessageWrap>>
        writer;
    std::unordered_set<std::shared_ptr<Client>> clients;
  };

  struct Reader {
    std::shared_ptr<
        apollo::cyber::Reader<apollo::cyber::message::PyMessageWrap>>
        reader;
    std::unordered_set<std::shared_ptr<Client>> clients;
  };

  typedef std::unordered_map<std::string, Writer> Writers;
  typedef std::unordered_map<std::string, Reader> Readers;
  Writers writers;
  Readers readers;
};
