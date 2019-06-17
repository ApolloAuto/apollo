# Apollo Cyber RT Terminologies

This page describes the definitions of the most commonly used terminologies in Cyber RT.

## Component

In an autonomous driving system, modules(like perception, localization, control systems...) exist in the form of components under Cyber RT. Each component communicates with the others through Cyber channels. The component concept not only decouples modules but also provides the flexibility for modules to be divided into components based individual module design.

## Channel

Channels are used to manage data communication in Cyber RT. Users can publish/subscribe to the same channel to achieve p2p communication.

## Task

Task is the abstract description of an asynchronous computation task in Cyber RT.

## Node

Node is the fundamental building block of Cyber RT; every module contains and communicates through the node. A module can have different types of communication by defining read/write and/or service/client in a node.

## Reader/Writer

Message read/write class from/to channel. Reader/Writer are normally created within a node as the major message transfer interface in Cyber RT.

## Service/Client

Besides Reader/writer, Cyber RT also provides service/client pattern for module communication. It supports two-way communication between nodes. A client node will receive a response when a request is made to a service.

## Parameter

Parameter service provides a global parameter access interface in Cyber RT. It's built based on the service/client pattern.

## Service discovery

As a decentralized design framework, Cyber RT does not have a master/central node for service registration. All nodes are treated equally and can find other service nodes through `service discovery`. `UDP` is used in Service discovery.

## CRoutine

Referred to as Coroutine concept, Cyber RT implemented CRoutine to optimize thread usage and system reource allocation.

## Scheduler

To better support autonomous driving scenarios, Cyber RT provides different kinds of resource scheduling algorithms for developers to choose from.

## Message

Message is the data unit used in Cyber RT for data transfer between modules.

## Dag file

Dag file is the config file of module topology. You can define components used and upstream/downstream channels in the dag file.

## Launch files

The Launch file provides a easy way to start modules. By defining one or multiple dag files in the launch file, you can start multiple modules at the same time.

## Record file

The Record file is used to record messages sent/received to/from channels in Cyber RT. Reply record files can help reproduce the behavior of previous operations of Cyber RT.

