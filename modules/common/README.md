# Common - Module

This module contains code that is not specific to any module but is useful for the functioning of Apollo.

## apollo_app

The **apollo_app** build target defines the abstract class ApolloApp,
which is implemented by all modules, as well as the macro `APOLLO_MAIN`,
used to launch each module.

## log

The **log** build target wraps the Google Log system into project-specific macros, allowing for more refined control over the levels of logging.


## macro

The **macro** build target defines a few commonly used class-specific macros.


## adapters

Adapters are used by the different modules to communicate with one another. The `AdapterManager` class hosts all the specific adapters and manages them.
Adapters need to be registered with the macro `REGISTER_ADAPTER`.
The Adapter class serves as a layer of abstraction between
Apollo modules and I/O mechanisms (e.g. ROS).

## configs/data

The vehicle configuration is specified in `configs/data`

## math

**math** implements a number of useful mathematical libraries.


## monitor

**Monitor** defines a logging system.

## proto

**Proto** defines a number of project-wide protocol buffers.

## status

**Status** is used for determining whether certain functions were performed successfully or not. If not, status provides helpful error messages.


## time

**Time** is a helper functions related to time.


## util

**Util** contains an implementation of a factory design pattern with registration, a few string parsing functions, and some utilities for parsing protocol buffers from files.


## vehicle_state

The **vehicle_state** class specifies the current state of the vehicle (e.g. position, velocity, heading, etc.).

