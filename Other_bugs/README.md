# Bugs Found from Our Mutated Environment

Below table presents details of bugs with root cause.
This shows our mutated environment discovers more bugs.
In the third column, * means bugs observed in the original environment (before mutation).
If the DCC shows difference, we consider they are unique swarm behavior (details are in Evaluation section).
However even if the swarm behaviors are different, the root causes are the same, we consider they are the same bugs.

| Alg. | Consequence (Behavior)        | Bug ID | Root cause                            |
| ---- | ----------------------------- | ------ | ------------------------------------- |
| A1   | Crash between neighbor drones | 1\*    | Missing collision detection           |
|      |                               | 2\*    | Naïve multi-force handling            |
|      |                               | 3      | Unsupported static movement           |
|      |                               | 4      | Excessive force in APF                |
|      | Crash into external objects   | 1\*    | Missing collision detection           |
|      |                               | 2\*    | Naïve multi-force handling            |
|      |                               | 3      | Unsupported static movement           |
|      |                               | 4      | Excessive force in APF                |
|      | Suspended progress            | 5      | Impossible global waypoint generation |
|      |                               | 6      | Wrong waypoint update                 |
|      |                               | 7\*    | Naïve swarm's pose measurement        |
|      |                               | 8      | Insensitive object detection          |
|      | Slow progress                 | 8      | Insensitive object detection          |
|      |                               |        |                                       |
| A2   | Crash between neighbor drones | 1      | Naïve/faulty avoiding method          |
|      |                               | 2      | Naïve multi-force handling            |
|      |                               | 3\*    | Excessive attractive force of goal    |
|      |                               | 4      | Insensitive object detection          |
|      | Crash into external objects   | 1      | Naïve/faulty avoiding method          |
|      |                               | 2      | Naïve multi-force handling            |
|      |                               | 3\*    | Excessive attractive force of goal    |
|      |                               | 4      | Insensitive object detection          |
|      |                               | 5      | Faulty detrouing method               |
|      | Slow progress                 | 6\*    | Drones detaching from a swarm         |
|      |                               |        |                                       |
| A3   | Crash between neighbor drones | 1\*    | Naïve detouring method                |
|      |                               | 2\*    | Excessive force in APF                |
|      |                               | 3\*    | Naïve multi-force handling            |
|      | Crash into external objects   | 2\*    | Excessive force in APF                |
|      |                               | 3\*    | Naïve multi-force handling            |
|      | Suspended progress            | 4      | Insensitive object detection          |
|      | Slow progress                 | 5      | Drones detaching from a swarm         |
|      |                               |        |                                       |
| A4   | Crash between neighbor drones | 1      | Naïve detouring method                |
|      |                               | 2      | Naïve multi-force handling            |
|      |                               | 3\*    | Missing collision detection           |
|      | Crash into external objects   | 1      | Naïve detouring method                |
|      |                               | 2      | Naïve multi-force handling            |
|      | Suspended progress            | 4      | Insensitive object detection          |