/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#pragma once

#include "clients.h"

#include <unordered_set>
#include <boost/asio.hpp>

class Node;

class Server : public std::enable_shared_from_this<Server>
{
public:
    Server(Node& node);
    ~Server();

    void run();

private:
    Node& node;
    Clients clients;

    boost::asio::io_service io;
    boost::asio::signal_set signals;

    boost::asio::ip::tcp::endpoint endpoint;
    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::ip::tcp::socket socket;

    void stop(const boost::system::error_code& error, int signal_number);

    void begin_accept();
    void end_accept(const boost::system::error_code& ec);
};
