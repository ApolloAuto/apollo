# Pandora_Apollo
## Clone
```
git clone https://github.com/HesaiTechnology/Pandora_Apollo.git --recursive
```

## Build
```
cd <project>
mkdir build ; cd build;
cmake .. ; make
```
## Add to your project 
### Cmake
```
add_subdirectory(<path_to>Pandora_Apollo)

include_directories(
	<path_to>Pandora_Apollo/include
	<path_to>Pandora_Apollo/src/Pandar40P/include
)

target_link_libraries(<Your project>
  Pandora
)

```
### C++
```
#include "pandora/pandora.h"
```

