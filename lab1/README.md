# Lab 1: TurtleSim Go To Goal Controller

## Success Criteria

- The turtle should move smoothly toward the goal position
- When it reaches the goal, it should stop completely (no wiggling)
- The controller should handle different goal positions reliably
- Move to 4 different position sequentally by prompt entering a new position after previous goal is reached

## Setup Instructions

### Prerequisites

- ROS2 installed on your system
- TurtleSim package

### Building the Project

1. Navigate to your workspace root
2. Build the package:

   ```
   colcon build
   ```

3. Source the setup script (this must be done in both terminals you use):

   ```
   source install/setup.bash
   ```

## Running the Project

### Terminal 1: Start TurtleSim

```
ros2 run turtlesim turtlesim_node
```

### Terminal 2: Run the Go-To-Goal Controller

```
ros2 run turtlesim_goal gotogoal.py
```

## Task Instructions

1. Examine the code in `src/turtlesim_goal/turtlesim_goal/gotogoal.py`
2. Find all instances of "FIXME" comments and fix the corresponding issues
3. The main problems likely involve:
   - Publisher/subscriber topic mixup
   - Distance calculation errors
   - Control logic issues for smooth navigation
   - Angular velocity control for proper orientation

## Testing Your Solution

1. After fixing the issues, rebuild the package using `colcon build`
2. Run the TurtleSim and controller
3. Input various goal coordinates when prompted
4. Observe if the turtle:
   - Moves directly to the goal
   - Stops precisely at the goal position
   - Doesn't oscillate or wiggle around the goal

## Troubleshooting

- If the turtle isn't moving at all, check the publisher/subscriber setup
- If the turtle moves but doesn't reach the goal, check the distance calculation
- If the turtle circles or oscillates around the goal, check the angular velocity control

Good luck!

## Hints

After starting turtlesim, you are able to see the names of topics and nodes

```bash
ros2 topic list
```

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
