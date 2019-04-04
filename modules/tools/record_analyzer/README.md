# Record Analyzer Tool

## Offline Record files analysis
### Functions

Record analyzer is a tool for analyzing the .record file created by cyber_recorder tool.

It currently supports statistical analysis for
 * Control module latency
 * Planning module latency
 * End to end system latency

 And distribution analysis for
 * Control error code
 * Control error message
 * Planning trajectory type
 * Planning estop
 * Planning error code
 * Planning error message

### Usage

```bash
python main.py -f record_file
```

## Simulation Score API

### Functions
This API is used for simulation to grade planning trajectories.

It currently supports following scores:
 * frechet_dist: calculate the frechet_dist for two consecutive planning trajectories
 * hard_brake_cycle_num: number of planning cycles that acceleration is less than -2.0 m/s^2
 * overall_score: aggregated score for above metrics

### Usage

```bash
python main.py -f record_file -s
```