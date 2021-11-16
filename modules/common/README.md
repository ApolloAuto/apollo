# Common - Module

This module contains code that is not specific to any module but is useful for the functioning of Apollo.

## adapters

Topics are used by different modules to communicate with one another. A large number of topic names are defined in `adapter_gflags`.

## configs/data

The vehicle configuration is specified in `configs/data`

## filters

**filters** implements some filter classes including DigitalFilter and MeanFilter.

## kv_db

**KVDB** is a lightweight key-value database to store system-wide parameters.

## latency_recorder

**LatencyRecorder** can record the latency between two time points.

## math

**math** implements a number of useful mathematical libraries.

## monitor_log

**Monitor** defines a logging system.

## proto

**Proto** defines a number of project-wide protocol buffers.

## status

**Status** is used for determining whether certain functions were performed successfully or not. If not, status provides helpful error messages.

## util

**Util** contains an implementation of a factory design pattern with registration, a few string parsing functions, and some utilities for parsing protocol buffers from files.

## vehicle_state

The **vehicle_state** class specifies the current state of the vehicle (e.g. position, velocity, heading, etc.).
