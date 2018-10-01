# Vehicle Modes

This folder contains different modes configuration for Apollo vehicle. Each mode
contains its runtime launch files to bring up all components.

## Name Convention

We'll convert the folder names to readable names automatically and display on
Dreamview. Please make it simple, clean and meaningful.

An example:

```text
# Folders with structure:
modes/
      mkz_standard/
                   drivers.launch
                   perception.launch
                   planning_control.launch

# will be displayed on Dreamview as:
Selectable mode:    "Mkz Standard"
Launchable modules: "Drivers", "Perception", "Planning Control"
```
