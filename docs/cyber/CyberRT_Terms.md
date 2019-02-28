This page describes the definitions of the most common used Cyber-RT terms.

- Component

	In autonomous driving system, modules(like perception, localization, control systems...) exist as a form of components under Cyber-RT. Each component communicate with each other through Cyber channels. The component concept not only decouples modules but also provides the flexibility for modules to be divided into components based individual module design.
- Channel

	Channels are used to manage the data communication in Cyber-RT. Users can publish/subscribe to the same channel to achieve p2p communication.
- Task

	Task is the abstract description of asynchronous computation task in Cyber-RT.
- Node 

	Node is the fundamental building block of Cyber-RT; every module contains a node and communicates through the node. A module can have different types of communication by defining read/write, service/client in a node. 
- Reader/Writer 

	Message read/write class from/to channel. Reader/Writer are normally created within a node as the major message transfer interface in Cyber-RT.
- Service/Client

	Besides reader/writer, Cyber-RT also provides service/client pattern for module communication. It supports two-way communication between nodes. A client node will receive a response when send a request to a service.
- Parameter

	Parameter service provides the global parameter access interface in Cyber-RT. It's built based on service/client pattern.
- Service discovery

	As a decentralized design framework, Cyber-RT doesn't have a master/center node for service registration. All nodes are treated equally and can find other service nodes through `service discovery`. `UDP` is used in Service discovery.
- CRoutine

	Referred to Coroutine concept, Cyber-RT implemented CRoutine to optimize thread usage and system reource allocation.
- Scheduler

	To better support autonomous driving scenarios, Cyber-RT provides different kinds of resource scheduling algorithms for developers to choose from.
- Messages

	Message is the data unit used in Cyber-RT for data transfer between modules. 
- Dag file

	Dag file is the config file of module topology. You can define components used, upstream/downstream channels in dag file.
- Launch files

	Launch file provides a easy way to start modules. By defining 1 or multiple dag files in the launch file, you can start multiple modules at the same time.
- Record file

	Record file is used to record messages sent/received to/from channels in Cyber-RT. Reply record files can help reproduce the behavior of earlier operation of Cyber-RT.

