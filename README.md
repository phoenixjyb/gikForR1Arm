# R1 Robot Arm GIK & Grasping Project

## Overview
This project provides a robust MATLAB-based workflow for simulating and planning grasping motions for the R1 robot arm. It features left-arm-only workspace sampling, collision avoidance (table and cardbox), and automated bottle placement to ensure feasible and safe grasps.

## Features
- **Generalized Inverse Kinematics (GIK)** for multi-stage grasping (approach, grasp, close)
- **Left arm workspace sampling** using `sampleReachableWorkspace` (torso and right arm locked)
- **Automated bottle placement**: The bottle is automatically moved to a reachable location if the initial pose is not feasible
- **Collision avoidance** with the table and cardbox
- **Visualization** of robot, bottle, table, cardbox, and trajectories
- **Clean, modular code** for easy extension and adaptation

## Branch Structure
- **master**: Clean, production-ready codebase
- **feature/robust-grasping-workflow**: Main development branch (recommended for further development)
- **backupFiles-only**: Legacy/experimental scripts and backup files (not used in main workflow)

## Setup
1. Clone the repository and open it in MATLAB.
2. Ensure all files are in the main project directory, with URDFs in `fineUrdfs/`.
3. Run `createAndPlaceBottleTable.m` to generate the bottle, table, and cardbox objects.
4. Run `simpleGIKprocess_R1BodyAndMug.m` to simulate the grasping process and visualize the results.

## Main Scripts
- **`createAndPlaceBottleTable.m`**: 
  - Loads the robot, samples the left arm workspace, and creates the bottle, table, and cardbox.
  - Checks if the bottle's grasp pose is reachable. If not, moves the bottle to the closest feasible point and verifies with IK.
- **`simpleGIKprocess_R1BodyAndMug.m`**:
  - Runs the three-stage GIK process (approach, grasp, close)
  - Animates the robot, bottle, table, and cardbox
  - Prints and saves final positions

## Workspace Sampling
- The function `sampleReachableWorkspace(robot, eeName, numSamples, visualize, jointsToSample)` samples random joint configurations for the **left arm only** (torso and right arm locked at home) and computes the reachable workspace of the end-effector.
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
- You can adapt the workflow for other objects by updating the creation and placement logic in `generateBottleAndTable.m`.
- The workspace sampling function can be used for any robot and end-effector by specifying the joints to sample.

## Legacy/Experimental Code
- The `backupFiles-only` branch contains legacy scripts and experimental code. It is not needed for the main workflow and is ignored in `master` and `feature/robust-grasping-workflow`.

## License
MIT License. See LICENSE file for details. 