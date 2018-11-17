# modules/common

```
Contains code that is not module-specific.
```

## apollo_app
```
The "apollo_app" build target defines the abstract class ApolloApp,
which is implemented by all modules, as well as the macro APOLLO_MAIN,
used to launch each module.
```

## log
```
The "log" build target wraps the Google Log system into project-specific macros,
allowing for more refined control over the levels of logging.
```

## macro
```
The "macro" build target defines a few commonly used class-specific macros.
```

## adapters
```
Adapters are used by the different modules to communicate with one another.
The AdapterManager class hosts all the specific adapters and manages them.
Adapters need to be registered with the macro REGISTER_ADAPTER.
The Adapter class serves as a layer of abstraction between
Apollo modules and I/O mechanisms (e.g. ROS).
```


## configs/data
```
These specify the vehicle configuration.
```

## math
```
Implements a number of useful mathematical libraries.
```

## monitor
```
Defines a logging system.
```

## proto
```
Defines a number of project-wide protocol buffers.
```

## status
```
Used for determining whether certain functions were performed successfully,
providing helpful error messages otherwise.
```

## time
```
Helper functions related to time.
```

## util
```
Contains an implementation of a factory design pattern with registration,
a few string parsing functions, and some utilities for parsing
protocol buffers from files.
```

## vehicle_state
```
This class specifies the current state of the vehicle (e.g. position, velocity,
heading, etc.).
```
