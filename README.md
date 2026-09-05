# PIDSIMULATOR

PID Simulator to help teach the concept of PID Control Engineering. This project is a small interactive simulator written for the Processing IDE that demonstrates a single-joint robot arm controlled by a PID controller. It includes realtime visualization of the arm, a time‑series plot (angle vs time), and a Bode‑plot / Nyquist analysis tool for frequency‑domain insight.

## What this is
A simple, interactive educational simulator that lets you experiment with PID gains and plant parameters for a pendulum/robot‑arm-like system and see the effects in both time and frequency domains.

### Stack
- **Language(s):** Processing (Java mode)
- **Runtime / environment:** Processing IDE (recommended)
- **Notable libraries / dependencies:** none external — uses Processing core APIs only

## How it's organized
```
PID_Controller_Visual_Example_Bode_V2.pde   # Main program (all code in one .pde)
README.md                                   # This file
Bode.png                                    # Example Bode screenshot
Nyquist-open.png                             # Nyquist open-loop example
Nyquist-closed.png                            # Nyquist closed-loop example
UI-full.png                                   # Full-window hero image
resp-step.png                                 # Time-response example
controls-closeup.png                           # Control panel close-up
```

How it fits together:
- The single .pde file contains the whole program: model, controller, plotting, UI, and the Bode/Nyquist analysis routine. The main draw loop applies the PID control law, steps the physical model (ArmSim), and updates/plots the results.

## Main components (in the code)
- ArmSim — lightweight physical model for the arm:
  - Parameters: arm length, arm mass, payload mass & distance, viscous drag, optional spring stiffness (tied to target angle), torque limit.
  - Methods: inertia(), gravity torque, step(torque, dt) for time integration, and impulse() for disturbances.
- PID — proportional/integral/derivative controller with a small derivative low‑pass filter (to stabilize the derivative term).
- Plot — rolling time plot for current angle and target (adaptive Y range over a 10 s window).
- BodePlot / runBode() — computes plant and closed‑loop frequency responses over a user range and renders magnitude (dB) and phase (deg). The plant is linearized about the current operating point and uses small‑signal stiffness.
- HUD, InputField, Button — simple UI widgets used in the left panel and overlays.
- Complex — tiny complex number helper used for frequency‑domain calculations.

## How the simulation runs (control loop)
Each frame (draw):
1. Read UI fields and set parameters.
2. Compute setpoint (either fixed target or a user toggled sine generator).
3. Compute error and derivative, evaluate PID: u = Kp*e + Ki*integral + Kd*derivative.
4. Apply actuator saturation (torque limit) and anti‑windup logic for the integral term.
5. Call sim.step(u, dt) which advances the arm's state.
6. Add samples to the time plot and redraw the arm, plots, and HUD.

## Controls and UI
- Left panel: editable fields for PID gains and plant parameters (Kp, Ki, Kd, Target, Arm Length, Arm Mass, Payload, Payload Distance, Drag, Spring stiffness, Torque limit).
- Setpoint generator: Sine Amp, Sine Freq, Sine Center. Toggle Sine ON/OFF with the "Sine" button.
- Buttons:
  - Step +10° / Step −10° — changes the target (or the sine center when the sine generator is active).
  - Sine: toggles the setpoint sine generator ON/OFF.
  - Run Bode: computes the frequency responses for the current parameters and draws the Bode plot.
  - Reset: reinitializes the simulation state and plots.
  - Zero Integral: zeroes the PID integral term.
  - Disturb (+Nm): applies a short impulse to the arm to test disturbance rejection.
- Mouse wheel: scroll the left control panel (when the mouse is over it).
- Keyboard: F toggles fill‑screen mode, Q quits.

## Bode analysis details
- When you click "Run Bode", the program does a small‑signal linearization about the current target angle and computes:
  - Plant G(s) = 1 / (I s^2 + b s + k) where I is rotational inertia, b is viscous drag, and k is small‑signal stiffness (spring − gravitational linearization term).
  - PID(s) assembled from Kp, Ki, Kd, and a derivative low‑pass (approximation derived from the PID.derivLPF setting).
  - Loop L(s) = PID(s)*G(s) and Closed‑loop T(s) = L(s) / (1 + L(s)).
- The Bode plot shows Plant (gray) and Closed‑loop (blue) magnitude (dB) and phase (deg).

## How to run
1. Install Processing: https://processing.org/ (use the latest stable release; Java 8+ compatible Processing builds are recommended).
2. Open Processing and open the file `PID_Controller_Visual_Example_Bode_V2.pde`.
3. Run the sketch (the ▶ play button in Processing).

Minimal commands (if you use the CLI or `processing-java`):
```
# from the repository root, assuming processing-java is installed and on PATH
processing-java --sketch=`pwd` --run
```

Notes:
- The sketch targets a fixed internal time step (1/120 s) for the physics simulation.
- The UI is built with simple hand‑rolled widgets inside the .pde; resizing the window reflows the layout.

## Suggested experiments for newcomers
- Start with Kp only: set Ki=0, Kd=0, and increase Kp until you see steady oscillation; observe rise time and overshoot.
- Add a small Ki to remove steady‑state error; observe integral windup if torque limit is low and test the Zero Integral button.
- Add Kd to reduce overshoot. Try toggling the derivative filter (pid.derivLPF in code) if you modify the source.
- Use the Disturb button to apply an impulse and see disturbance rejection.
- Run Bode and sweep frequencies to see where the closed‑loop gain drops and how phase margin appears.

## Screenshots and images
Screenshots are very helpful for newcomers (UI layout, time‑response example, and frequency‑domain plots). Images have been added to the repository; examples below are embedded at full resolution for clarity.

UI / full window :

![Full UI window](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/UI-full.png)

"PID Simulator — controls, realtime time-plot, Bode & Nyquist panels, and visual arm."

Time response (step / sine):

![Time response (step)](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/resp-step.png)

"Time response — step / sine response showing system behavior and HUD."

Bode (magnitude & phase):

![Bode: plant and closed-loop](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/Bode.png)

"Bode plot: plant (gray) and closed-loop (blue) magnitude & phase, linearized at θ = 30°." 

Nyquist (open-loop):

![Nyquist open-loop](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/Nyquist-open.png)

"Nyquist (open-loop) — classical -1 + j0 point marked."

Nyquist (closed-loop):

![Nyquist closed-loop](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/Nyquist-closed.png)

"Nyquist (closed-loop) — shows T(jω) loops."

Control panel close-up:

![Control panel close-up](https://raw.githubusercontent.com/melzein1/PIDSIMULATOR/main/controls-closeup.png)

 "Control panel: PID and plant parameters (edit and run)."


