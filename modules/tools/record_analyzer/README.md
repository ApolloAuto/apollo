# Record Analyzer Tool

## Functions

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

## Usage

```bash
python main.py -f record_file
```
