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

## Main Steps and Workflow (simpleGIKprocess_R1BodyAndMug.m)
This script demonstrates a complete pipeline for planning and executing a collision-aware grasp of a bottle using the R1 robot. The main steps are:

1. **Initialization and Setup**: Load robot and environment objects, adjust bottle pose, set initial robot configuration.
2. **Environment and Collision Setup**: Attach table and cardbox as collision bodies, define protected robot links, and set up distance constraints.
3. **Inverse Kinematics (IK) and Reachability**: Set up GIK solver, define target and approach poses, check reachability with the left arm.
4. **Torso Adjustment for Reachability**: If unreachable, sample torso joint configurations to enable reach.
5. **GIK-based Grasp Planning**: Lock all joints except left arm and gripper, solve for approach, grasp, and close stages with collision constraints.
6. **Collision Checking**: Check for collisions at start and grasp configurations, visualize and report if any are found.
7. **Motion Planning**: Use RRT to plan a collision-free path from start to grasp, considering all obstacles.
8. **Final Pose and Metrics**: Print and compare final positions of gripper, fingers, and bottle.
9. **Animation and Visualization**: Animate the robot's motion, export videos, and plot trajectories.
10. **Data Export**: Save final configuration and trajectory as .mat and .json files.
11. **Helper Functions**: Includes functions for torso solving, reachability sampling, and multi-stage GIK.
12. **Additional Collision Checks**: Check for collisions at every step of the planned path.

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