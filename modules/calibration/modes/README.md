# Vehicle Modes

This folder contains different modes configuration for Apollo vehicle. Each mode
contains its runtime launch files to bring up all components.

## Name Convention

We'll convert the folder names to readable names automatically and display on
Dreamview. Please make it simple, clean and meaningful.

An example mode files structure:

```text
modes/
      mkz_standard/
                   close_loop.launch
                   map_collection.launch
                   ...
```

On Dreamview it can be selected in drop down menu:
```text
Mode:    "Mkz Standard"
Config:  "Close Loop" or "Map Collection"
```
