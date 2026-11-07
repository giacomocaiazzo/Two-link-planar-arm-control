# Usage

## Complete study

Open the repository folder as MATLAB's Current Folder, then open [Control_Study.mlx](../Control_Study.mlx). Begin with **System matrices** and execute the sections in order up to the control method you want to explore.

The Live Script brings together modeling, controller design, state estimation and discrete-time analysis.

## Individual methods

Start each method with a fresh workspace. Execute the script sections in order and keep the calculated variables in the workspace while using its Simulink diagram.

### Pole placement

Run the entire [Pole_placement.m](../pole-placement/Pole_placement.m) script. Then open [Pole_placement_simulink.slx](../pole-placement/Pole_placement_simulink.slx) and start the simulation.

The script covers state feedback, reference precompensation, integral action and ITAE pole placement.

### LQR and LQG

Run the entire [LQR.m](../lqr-lqg/LQR.m) script. Then open [LQR_simulation.slx](../lqr-lqg/LQR_simulation.slx) and start the simulation.

The script explores state and input weights, integral action, prescribed decay rates and state estimation.

### H-infinity control

Open [H-infinity_control.m](../mixed-sensitivity/H-infinity_control.m) and execute its sections in order to define the weights, synthesize the controller and inspect the sensitivity functions.

### Discrete-time control

In [Pole_placement_DT.m](../discrete-control/Pole_placement_DT.m), execute through **Closed-loop interconnection with integral action** for the pole-placement design. Then open [Pole_placement_simulation_DT.slx](../discrete-control/Pole_placement_simulation_DT.slx) and start the simulation.

The remaining script sections explore discrete-time LQR and reference precompensation.

## Technical report

See [Two_link_planar_arm_control.pdf](Two_link_planar_arm_control.pdf) for the mathematical model, control-design rationale and simulation results.
