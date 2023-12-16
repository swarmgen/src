# Prototype: Adaptive Swarm
This example includes A1-1 that we addressed in the paper.

## Quick start
```bash
$ python src/planner.py adswarm -k vanilla -rs <seed rand num start> -re <seed rand num end>
```

## Result
Result files are stored in `output` folder.
`contribution.log` has DCC's raw delta data.

### Clean-up and store the output data

```
./output_store.sh <name of data folder>
```
It will make a folder and move all files in `output` folder to it.

```
./output_cleanup
```
It will move all files in `output` folder to `data/deprecated`.

## Experiment configuration for new mission
Below code in `conf.py` contains general configuration for the new mission.

```python
'''
Environment configuration
'''
MAP_SIZE = 1000 # when you want to create the new map, need to change below configurations.
MAP_BOUNDS_METER = 5.0

MAP_SIZE_new_x = 400
MAP_SIZE_new_y = 2000
MAP_BOUNDS_METER_new_x = 2.0
MAP_BOUNDS_METER_new_y = 10.0

XY_START = np.array([1.2, 9.0]) # define starting point
XY_GOAL = np.array([-1.0, -6.0]) # define goal of the mission

```
- Obstacle: `obstacle.py`.


