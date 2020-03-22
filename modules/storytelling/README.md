# Storytelling

## Introduction
Storytelling is a global and high-level Scenario Manager to help coordinate cross-module actions. In order to safely operate the autonomous vehicle on urban roads, complex planning scenarios are needed to ensure safe driving. These complex scenarios may involve different modules to ensure proper maneuvering. In order to avoid a sequential based approach to such scenarios, a new isolated scenario manager, the "Storytelling" module was created. This module creates stories which are complex scenarios that would trigger multiple modules' actions. Per some predefined rules, this module creates ne or multiple stories and publishes to
`/apollo/storytelling` channel. The main advantage of this module is to fine tune the driving experience and also isolate complex scenarios packaging them into stories that can be subscribed to by other modules like Planning, Control etc. 

## Input

* Localization
* HD Map

## Output

* Story - which can be subscribed to by other modules 

## Sample Story

Please refer to [CloseToJunction](https://github.com/ApolloAuto/apollo/tree/master/modules/storytelling/story_tellers) sample scenario to understand the Storytelling module

```
Note:
The base_teller.h is a virtual class to help you implement your own story. Please inherit this class when writing your own story.
```