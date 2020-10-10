# Video Records Restoring Tool

## Prerequisite

Get into Apollo Docker
Setup LD_LIBRARY_PATH by running following command

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/apollo/ffmpeg/lib
```

## Restore

This tool converts video frames from source record file into images and then restores them to the generated record file.

```bash
python restore_video_record.py --from_record=<src record file> --to_record=<dst record file>
```
