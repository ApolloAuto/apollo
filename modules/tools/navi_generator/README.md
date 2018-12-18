# NaviGenerator

## Introduction
NaviGenerator is a tool for recording, editing, and displaying navigation lines. Navigation lines are the most critical data for generating real-time relative maps.

## Features 
- Collect vehicle trajectories in real time.
- Generate navigation lines based on vehicle trajectories.
- Correct a specific portion of a navigation line.
- Edit speed limits for a navigation line.
- Generate a lane-level navigation routing based on given starting and end points .

## Usage

1. Start the background server:

```bash
# In docker and apollo root dir
bash scripts/navi_generator.sh 
```
2. Open the URL: http://localhost:9888 in a Chrome or FireFox browser, and perform related tasks on the web page.

3. Stop the background server:

```bash
# In docker and apollo root dir
bash scripts/navi_generator.sh stop
```