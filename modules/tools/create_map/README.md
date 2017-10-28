# Map Creation Tool

## Lane Recorder

This tools records Localization trajectory and Mobileye lane detection information into a csv file (default file path: /tmp/lane.csv).

Run the following command from your Apollo root dir:

```bash
python modules/tools/create_map/lane_recorder.py
```

## Map Creator

This tool reads information from a csv file (default file path: /tmp/lane.csv) and create a map (designated by map_name) with a center lane and `left_lanes` left lanes and `right_lanes` right lanes.

```bash
./scripts/create_map.sh map_name [left_lanes right_lanes]
```
