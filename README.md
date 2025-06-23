# R1 Robot Arm GIK & Grasping Project

## Overview
This project provides a MATLAB-based workflow for simulating and planning grasping motions for the R1 robot arm. It features robust workspace sampling, collision avoidance, and automated bottle placement to ensure feasible and safe grasps.

## Features
- **Generalized Inverse Kinematics (GIK)** for multi-stage grasping (approach, grasp, close).
- **Workspace sampling** using the unified `sampleReachableWorkspace` function.
- **Automated bottle placement**: The bottle is automatically moved to a reachable location if the initial pose is not feasible.
- **Collision avoidance** with the table and environment.
- **Visualization** of robot, bottle, table, and trajectories.
- **Modular code** for easy extension and adaptation to other objects or robots.

## Setup
1. Clone the repository and open it in MATLAB.
2. Ensure all files are in the main project directory, with URDFs in `fineUrdfs/`.
3. Run `loadRobotBottleTable.m` to generate the bottle and table objects.
4. Run `simpleGIKprocess_R1BodyAndMug.m` to simulate the grasping process and visualize the results.

## Main Scripts
- **`loadRobotBottleTable.m`**: 
  - Loads the robot, samples the workspace, and creates the bottle and table.
  - Checks if the bottle's grasp pose is reachable. If not, moves the bottle to the closest feasible point and verifies with IK.
- **`simpleGIKprocess_R1BodyAndMug.m`**:
  - Runs the three-stage GIK process (approach, grasp, close).
  - Animates the robot, bottle, and table.
  - Prints and saves final positions.

## Workspace Sampling
- The function `sampleReachableWorkspace(robot, eeName, numSamples, visualize)` samples random joint configurations within joint limits and computes the reachable workspace of the end-effector.
- Handles continuous joints and invalid samples robustly.
- Can optionally visualize the workspace.

## Automated Bottle Placement
- After sampling the workspace, the script checks if the bottle's grasp pose is within 5 cm of any sampled point.
- If not, the bottle is moved to the closest sampled point, and IK is used to verify reachability.
- The bottle is only updated if IK succeeds, ensuring robust and feasible placement.

## Troubleshooting
- If you see a warning that the bottle is not reachable, the script will attempt to move it automatically.
- Ensure all required files (URDFs, helper functions) are in the correct locations.
- For custom objects or new robots, update the relevant scripts and URDF paths.

## Extending the Project
- You can adapt the workflow for other objects by updating the creation and placement logic in `createBottleAndTable.m`.
- The workspace sampling function can be used for any robot and end-effector.

## License
MIT License. See LICENSE file for details. 