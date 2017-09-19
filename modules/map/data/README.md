# Map Data

A bundle of correlated maps are organized as a directory, with a structure like:

```txt
<map_dir>                        # Defined by FLAGS_map_dir
  |-- base_map.xml               # Defined by FLAGS_base_map_filename
  |-- routing_map.bin            # Defined by FLAGS_routing_map_filename
  |-- sim_map.bin                # Defined by FLAGS_sim_map_filename
  |-- default_end_way_point.txt  # Defined by FLAGS_end_way_point_filename
```

You can specify the map filenames as a list of candidates:

```txt
--base_map_filename="base.xml|base.bin|base.txt"
```

Then it will find the first available file to load. Generally we follow the
extension pattern of:

```txt
x.xml  # An OpenDrive formatted map.
x.bin  # A binary pb map.
x.txt  # A text pb map.
```

## Use a different map

1. [Prefered] Change global flagfile: *modules/common/data/global_flagfile.txt*

   Note that it's the basement of all modules' flagfile, which keeps the whole
   system in one page.

1. Pass as flag, which only affect individual process:

   ```bash
   <binary> --map_dir=/path/to/your/map
   ```

1. Override in the module's own flagfile, which generally located at
   *modules/<module_name>/conf/<module_name>.conf*

   Obviously it also only affect single module.

   ```txt
   --flagfile=modules/common/data/global_flagfile.txt

   # Override values from the global flagfile.
   --map_dir=/path/to/your/map
   ```
