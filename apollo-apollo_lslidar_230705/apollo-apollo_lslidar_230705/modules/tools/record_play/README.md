# Record and Play Tool

## Prerequisite

Run the following command from your Apollo root dir:

```bash
bash apollo.sh build
source scripts/apollo_base.sh
```

## Recorder

This tool records trajectory information from gateway into a csv file, the file
name is defined in filename_path.

## Player

This tool reads information from a csv file and publishes planning trajectory in
the same format as real planning node.

Default enable recorder and play under `/apollo` utilizing:

```bash
bash scripts/rtk_recorder.sh
```

and

```bash
bash scripts/rtk_player.sh
```

Or using button on Dreamview
