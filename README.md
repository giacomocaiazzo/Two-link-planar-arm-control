# Two-Link Planar Arm Control

A MATLAB and Simulink project on reference tracking and state estimation for a two-link planar manipulator. The study compares pole placement, ITAE, LQR/LQG, H-infinity mixed-sensitivity design and discrete-time control using a linearized model of the arm.

## Project files

| File | Description |
| --- | --- |
| [Control_Study.mlx](Control_Study.mlx) | General Live Script covering the complete study, from the system model to the different control strategies. |
| [Pole_placement.m](pole-placement/Pole_placement.m) | Continuous-time pole placement, reference precompensation, integral action, ITAE and Luenberger observer design. |
| [Pole_placement_simulink.slx](pole-placement/Pole_placement_simulink.slx) | Simulink diagram for the pole-placement designs. |
| [LQR.m](lqr-lqg/LQR.m) | LQR tuning, integral action, prescribed decay rate and Kalman state estimation. |
| [LQR_simulation.slx](lqr-lqg/LQR_simulation.slx) | Simulink diagram for LQR and LQG experiments. |
| [H-infinity_control.m](mixed-sensitivity/H-infinity_control.m) | Mixed-sensitivity controller synthesis and sensitivity analysis. |
| [Pole_placement_DT.m](discrete-control/Pole_placement_DT.m) | Plant discretization, discrete-time pole placement and discrete LQR. |
| [Pole_placement_simulation_DT.slx](discrete-control/Pole_placement_simulation_DT.slx) | Simulink diagram for the discrete-time designs. |
| [Two_link_planar_arm_control.pdf](docs/Two_link_planar_arm_control.pdf) | Technical report with mathematical derivations, simulation results and discussion (Italian). |

## How to use

1. Open [Control_Study.mlx](Control_Study.mlx) in the Live Editor and execute its sections to explore the complete study.
2. For a specific method, use the script in its topic folder. Calculate the design variables before running the corresponding Simulink diagram.
3. Read the [usage guide](docs/USAGE.md) for the execution sequence of each method.
