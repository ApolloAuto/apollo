# Map Show

## About

Mapshow is a tool to display hdmap info on a map.

## Setup

If you run mapshow inside docker, there is no setup for running the tool.
Otherwise, you have to run following command to setup python path.

```bash
# In apollo root dir:
source scripts/apollo_base.sh
```

## Usage

> usage: python mapshow.py \[-h] -m MAP \[-sl] [-l LANEID [LANEID ...]]
>
> optional arguments:
>
>  -h, --help            show this help message and exit
>
>  -m MAP, --map MAP     Specify the map file in txt or binary format
>
>  -sl, --showlaneids    Show all lane ids in map
>
>  -l, --laneid    Show specific lane id(s) in map
>
>  -l LANEID \[LANEID ...], --laneid LANEID \[LANEID ...]  Show specific lane id(s) in map
>
>  -signal, --showsignals    Show all signal light stop lines with ids in map
>
>  -stopsign, --showstopsigns    Show all stop sign stop lines with ids in map
>
>  -junction, --showjunctions    Show all pnc-junctions with ids in map
>

## Examples

Show basic map layout only

```bash
python mapshow.py -m /path/to/map/file
```

Show basic map layout with all lane ids

```bash
python mapshow.py -m /path/to/map/file -sl
```

show basic map layout with specific lane ids

```bash
python mapshow.py -m /path/to/map/file -l 1474023788152_1_-1
```