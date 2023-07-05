# MapViewers

MapViewers folder contains a set of tools for plotting map related data.

### gmapviewer.py

Gmapviewer is a tool to display HDMap on Google map through Google map javascript API. It generates a html file (gmap.html) with HDMap data. You could view the data by opening the html file in a web browser.

You need to install yattag before running the script

```
pip install yattag
```

Inside docker, run the following command from your Apollo root dir:

```
python modules/tools/mapviewer/gmap_viwer.py -m map_path_and_file
```

An output file

```
gmap.html
```

can be found in your Apollo root dir.

The default utm zone is set to ***10*** in this tool. if the HDMap is located in a differet utm zone, run following command with a specified zone id:

```
python modules/tools/mapviewer/gmap_viwer.py map_path_and_file utm_zone_id
```

### hdmapviewer.py
Activate environments:

```
conda create --name py27bokeh python=2.7.15 numpy scipy bokeh protobuf
conda activate py27bokeh
```

Inside docker, run the following command from Apollo root dir:
```
python modules/tools/mapviewer/hdmapviwer.py -m map_path_and_file
```

An output file

```
hdmap.html
```

can be found in your Apollo root dir.


Deactivate environments:
```
conda deactivate
```
