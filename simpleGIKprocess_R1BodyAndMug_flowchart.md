# SimpleGIKprocess_R1BodyAndMug.m - Program Flowchart

## Overview
This MATLAB script implements a three-stage Generalized Inverse Kinematics (GIK) solution for an R1 robot arm to grasp a bottle using the left gripper. The process includes reachability checking, torso adjustment if needed, and a complete grasping sequence.

## Main Program Flow

```mermaid
flowchart TD
    A[Start: Close all figures] --> B[Load bottle.mat]
    B --> C[Load table.mat]
    C --> D[Import robot URDF]
    D --> E[Set end-effector name]
    E --> F[Initialize robot configuration]
    F --> G[Set finger joints to open position 0.05]
    G --> H[Add table as obstacle]
    H --> I[Create distance constraints]
    I --> J[Setup GIK solver]
    J --> K[Define approach pose]
    K --> L[Define joint groups]
    L --> M[Extract grasp position]
    M --> N[Perform GIK reachability check]
    
    N --> O{Is target reachable?}
    O -->|Yes| P[Lock torso and right arm]
    O -->|No| Q[Start torso adjustment]
    
    P --> R[Call solveGraspGIK function]
    Q --> S[Sample torso poses]
    S --> T[Test each torso sample]
    T --> U{Found reachable pose?}
    U -->|No| V[Error: Target unreachable]
    U -->|Yes| W[Update initial guess]
    W --> X[Lock non-essential joints]
    X --> R
    
    R --> Y[Display GIK status for each stage]
    Y --> Z[Print final positions]
    Z --> AA[Animate trajectory]
    AA --> BB[Save trajectory to JSON]
    BB --> CC[End]
    
    V --> CC
```

## Detailed Function Flow

### 1. Initialization Phase
```mermaid
flowchart LR
    A[Load bottle.mat] --> B[Move bottle to target position]
    B --> C[Compute grasp pose offset]
    C --> D[Load table.mat]
    D --> E[Import robot URDF]
    E --> F[Set end-effector: left_gripper_link]
    F --> G[Initialize home configuration]
    G --> H[Update with R1 whole body positions]
    H --> I[Set finger joints to 0.05 open]
```

### 2. Environment Setup
```mermaid
flowchart LR
    A[Create table obstacle] --> B[Add collision geometry]
    B --> C[Attach to robot base]
    C --> D[Define links to protect]
    D --> E[Create distance constraints]
    E --> F[2cm clearance from table]
```

### 3. Reachability Check
```mermaid
flowchart TD
    A[Setup GIK checker] --> B[Set loose tolerances]
    B --> C[Lock non-left-arm joints]
    C --> D[Solve GIK for reachability]
    D --> E{Success?}
    E -->|Yes| F[Proceed with fixed torso]
    E -->|No| G[Start torso adjustment]
```

### 4. Torso Adjustment (if needed)
```mermaid
flowchart TD
    A[Define torso joint ranges] --> B[Create 5x5 grid of samples]
    B --> C[Generate torso configurations]
    C --> D[For each sample]
    D --> E[Generate reachability workspace]
    E --> F{Within 15cm of target?}
    F -->|Yes| G[Mark as reachable]
    F -->|No| H[Mark as unreachable]
    G --> I{More samples?}
    H --> I
    I -->|Yes| D
    I -->|No| J{Found reachable?}
    J -->|Yes| K[Use best torso pose]
    J -->|No| L[Error: Unreachable]
```

### 5. solveGraspGIK Function
```mermaid
flowchart TD
    A[Setup GIK solver] --> B[Define approach pose 10cm above]
    B --> C[Create pose constraints]
    C --> D[Find finger joint indices]
    D --> E[Lock fingers to open position 0.05]
    E --> F[Solve Stage 1: Approach]
    F --> G[Solve Stage 2: Grasp]
    G --> H[Calculate finger closing]
    H --> I[Set finger positions for grasp]
    I --> J[Generate trajectory interpolation]
    J --> K[Return all configurations]
```

### 6. Finger Closing Calculation
```mermaid
flowchart LR
    A[Get bottle diameter] --> B[Calculate initial finger gap]
    B --> C[Set target gap = bottle diameter]
    C --> D[Calculate closing distance]
    D --> E[Compute finger joint position]
    E --> F[Clamp to joint limits 0-0.05]
    F --> G[Apply to finger joints]
```

### 7. Trajectory Generation
```mermaid
flowchart LR
    A[Stage 1: 50 points] --> B[initialGuess → qApproach]
    B --> C[Stage 2: 50 points]
    C --> D[qApproach → qGrasp]
    D --> E[Stage 3: 30 points]
    E --> F[qGrasp → qClose]
    F --> G[Concatenate all stages]
```

### 8. Animation and Output
```mermaid
flowchart TD
    A[Display GIK status] --> B[Print final positions]
    B --> C[Create 3D visualization]
    C --> D[Animate through trajectory]
    D --> E[Show final closed state]
    E --> F[Save trajectory to JSON]
```

## Key Variables and Parameters

### Robot Configuration
- **Robot**: R1 v2.1.0 from `fineUrdfs/r1_v2_1_0.urdf`
- **End-effector**: `left_gripper_link`
- **Finger joints**: `left_gripper_finger_joint1`, `left_gripper_finger_joint2`
- **Finger limits**: 0.0 (closed) to 0.05 (open)

### Grasping Parameters
- **Approach offset**: 10cm above grasp pose
- **Table clearance**: 2cm minimum distance
- **Torso adjustment range**: ±0.3 radians for joints 1&2
- **Reachability tolerance**: 15cm for torso adjustment

### Trajectory Parameters
- **Stage 1 (Approach)**: 50 interpolation points
- **Stage 2 (Grasp)**: 50 interpolation points  
- **Stage 3 (Close)**: 30 interpolation points
- **Animation pause**: 0.5 seconds per frame

## Helper Functions

### 1. `solveTorsoToHelpReach()`
- **Purpose**: Solve GIK for torso adjustment
- **Inputs**: Robot, target pose, joint lists, end-effector name
- **Outputs**: Success flag, new torso configuration

### 2. `generateReachabilityWithTorso()`
- **Purpose**: Generate reachability workspace for given torso configuration
- **Inputs**: Robot, torso config, joint list, end-effector name
- **Outputs**: Array of end-effector positions

### 3. `solveGraspGIK()`
- **Purpose**: Main GIK solving function for three-stage grasping
- **Inputs**: Robot, end-effector, grasp pose, constraints, initial guess, joint bounds, bottle diameter
- **Outputs**: Three configurations (approach, grasp, close), trajectory, solution info

## Error Handling

1. **Joint validation**: Checks if defined joints exist in robot model
2. **Reachability**: Tests if target is reachable before proceeding
3. **Torso adjustment**: Handles cases where fixed torso is insufficient
4. **GIK convergence**: Reports best available solution if not fully successful
5. **Finger joints**: Warns if finger joints are not found

## Output Files

- **traj_exmaple.json**: Trajectory data in JSON format
- **Console output**: GIK status, final positions, diagnostic information
- **3D visualization**: Animated robot movement and final state

## Dependencies

### Required Files
- `bottle.mat`: Bottle object with pose and collision geometry
- `table.mat`: Table object with collision geometry
- `fineUrdfs/r1_v2_1_0.urdf`: Robot URDF model
- `updateHomePositionforR1_wholeBody.m`: Robot configuration function

### MATLAB Toolboxes
- Robotics System Toolbox
- Computer Vision Toolbox (for transformations)
- Parallel Computing Toolbox (for parfor loops)

---

*This flowchart documents the complete program flow for the R1 robot arm bottle grasping implementation using Generalized Inverse Kinematics.* 