# SimpleGIKprocess_R1BodyAndMug.m - Program Flowchart

## Overview
This MATLAB script implements a robust, modular three-stage Generalized Inverse Kinematics (GIK) solution for the R1 robot arm to grasp a bottle using the left gripper. The process includes left-arm-only reachability checking (torso and right arm locked), cardbox and table collision integration, and a complete grasping sequence.

---

## Key Workflow Updates (2024)
- **Left arm only**: Workspace sampling and GIK are performed with only the left arm joints free; torso and right arm are locked at home.
- **Cardbox integration**: Cardbox is created, visualized, and included in collision constraints alongside the table.
- **No backup/legacy code**: All main scripts and functions are clean and robust; legacy/experimental code is in the `backupFiles-only` branch.

---

## Main Program Flow

```mermaid
flowchart TD
    A[Start: Close all figures] --> B[Load bottle.mat]
    B --> C[Load table.mat]
    C --> D[Import robot URDF]
    D --> E[Set end-effector name]
    E --> F[Initialize robot configuration]
    F --> G[Set finger joints to open position 0.05]
    G --> H[Add table and cardbox as obstacles]
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

---

## Notable Implementation Details
- **Workspace sampling**: Uses `sampleReachableWorkspace` with only left arm joints free.
- **Environment setup**: Both table and cardbox are added as fixed obstacles and included in collision constraints.
- **Automated bottle placement**: If the initial grasp pose is unreachable, the bottle is moved to the closest feasible point.
- **Trajectory and animation**: Three-stage GIK (approach, grasp, close) with full 3D and top-down animation, and trajectory export.
- **Clean codebase**: No references to backup or legacy scripts; all such code is in the `backupFiles-only` branch.

---

## Branch Structure
- **master**: Clean, production-ready codebase
- **feature/robust-grasping-workflow**: Main development branch (recommended for further development)
- **backupFiles-only**: Legacy/experimental scripts and backup files (not used in main workflow)

---

*This flowchart documents the complete, robust program flow for the R1 robot arm bottle grasping implementation using Generalized Inverse Kinematics. For legacy or experimental code, see the `backupFiles-only` branch.* 