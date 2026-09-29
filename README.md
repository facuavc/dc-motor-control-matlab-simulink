# DC motor position control with MATLAB and Simulink

Modeling and PID control of a DC motor driving a spring-damper-loaded rotating bar. This academic project combines an electromechanical model, frequency-domain analysis and closed-loop position simulation.

Developed by **Facundo Aguilera Van Cauwlaert** for **Control of Mechanical Systems**, semester **2025/2026**.

**[Read the technical report](Report%204%20-%20DC%20Motor.pdf)**

## Project overview

The objective is to regulate the angular position of a mechanical load through the motor armature voltage. The model accounts for armature resistance and inductance, motor torque, back electromotive force, mechanical inertia, damping, spring stiffness and linearized gravity effects.

The study includes:

- Derivation of the voltage-to-angle transfer function.
- Analysis of plant poles and open-loop poles and zeros.
- PID tuning through frequency-response and root-locus analysis in the report.
- Closed-loop step-response metrics and a Simulink implementation of the controller.

## Repository contents

| File | Purpose |
| --- | --- |
| [`DCMotor.m`](DCMotor.m) | Defines parameters, builds the plant and PID transfer functions, evaluates poles and step-response metrics, plots the asymptotic Bode diagram and runs Simulink. |
| [`simulationDCMotor.slx`](simulationDCMotor.slx) | Unity-feedback position-control model with separate proportional, integral and derivative branches. |
| [`asymp.m`](asymp.m) | Included helper for Bode plots with magnitude and phase asymptotes. |
| [`Report 4 - DC Motor.pdf`](Report%204%20-%20DC%20Motor.pdf) | Model derivation, controller comparisons and reported response metrics. |

## Mathematical model

For the angular coordinate $\theta$, the equivalent mechanical parameters are

$$
J_{\mathrm{eq}}=J+\frac{mL^2}{4},\qquad
b_{\mathrm{eq}}=rL^2,\qquad
k_{\mathrm{eq}}=kL^2-\frac{mgL}{2}.
$$

The linearized mechanical and electrical equations implemented in the code are

$$
J_{\mathrm{eq}}\ddot{\theta}+b_{\mathrm{eq}}\dot{\theta}+k_{\mathrm{eq}}\theta=k_\phi i_a,
$$

$$
L_a\dot{i}_a+R_a i_a=V_a-k_\phi\dot{\theta}.
$$

The same numerical constant $k_\phi$ is used for torque and back EMF in consistent SI units. The voltage-to-angle transfer function is

$$
G(s)=\frac{\Theta(s)}{V_a(s)}=
\frac{k_\phi}{(J_{\mathrm{eq}}s^2+b_{\mathrm{eq}}s+k_{\mathrm{eq}})(L_as+R_a)+k_\phi^2s}.
$$

The controller and closed-loop transfer function are

$$
R(s)=K_p\left(1+T_ds+\frac{1}{T_is}\right),\qquad
T(s)=\frac{R(s)G(s)}{1+R(s)G(s)}.
$$

The equations above follow `DCMotor.m`; the electrical equation printed in the report contains typographical inconsistencies.

## Default parameters

| Parameter | Value |
| --- | ---: |
| Load mass, `m` | 80 kg |
| Inertia parameter, `J` | 20 kg·m² |
| Bar length, `L` | 1 m |
| Spring stiffness, `k` | 500 N/m |
| Damping coefficient, `r` | 25 N·s/m |
| Gravity, `g` | 9.81 m/s² |
| Armature inductance, `La` | 0.001 H |
| Armature resistance, `Ra` | 0.1 Ω |
| Motor torque constant, `k_phi` | 0.0001 N·m/A |
| Proportional gain, `kp` | 1000 |
| Integral time, `Ti` | 1 s |
| Derivative time, `Td` | 0.5 s |
| Reference angle, `theta_ref` | 1 rad |

These values give $J_{\mathrm{eq}}=40$ kg·m², $b_{\mathrm{eq}}=25$ N·m·s/rad and $k_{\mathrm{eq}}=107.6$ N·m/rad. In parallel PID form, $K_i=K_p/T_i=1000$ and $K_d=K_pT_d=500$.

## Requirements

- MATLAB and Simulink. The model was saved in **R2025b**; earlier releases may require model export to a previous version.
- Control System Toolbox for functions including `tf`, `series`, `feedback`, `pole`, `zero`, `stepinfo` and `bode`.

Keep all repository files in the same working folder. No Simscape blocks are used in the supplied model.

## Run the project

Download or clone the repository:

```bash
git clone https://github.com/facuavc/dc-motor-control-matlab-simulink.git
cd dc-motor-control-matlab-simulink
```

Set this folder as MATLAB's Current Folder, then run:

```matlab
DCMotor
```

The script clears the workspace and closes existing figures. It creates the transfer functions, prints analysis results, plots the open-loop Bode diagram and runs `simulationDCMotor.slx`.

To inspect the block diagram:

```matlab
open_system('simulationDCMotor.slx')
```

Run the script before simulating the model independently: the blocks depend on workspace variables such as `G_num`, `G_den`, `kp`, `Ti`, `Td` and `theta_ref`.

### Plot the simulated position

After simulation, use the logged timeseries directly:

```matlab
figure;
plot(sim_out.position.Time, sim_out.position.Data, 'LineWidth', 1.5);
hold on;
yline(theta_ref, '--', 'Reference');
grid on;
xlabel('Time [s]');
ylabel('Angular position [rad]');
title('Closed-loop position response');
legend('Position', 'Reference', 'Location', 'best');
```

If the final line of `DCMotor.m` raises a property-name error, replace `sim_out.position.data` with the standard timeseries property `sim_out.position.Data`.

### Simulation duration and additional plots

The saved model has a fixed stop time of **600 s**. Although the script defines `sT = 1000`, its existing `sim` call does not pass this value to the model. To explicitly simulate for `sT` seconds, replace that call with:

```matlab
sim_out = sim('simulationDCMotor.slx', 'StopTime', num2str(sT));
```

The margin, Nyquist and root-locus commands are commented out in the script. Uncomment them, or run the following after the transfer functions have been created:

```matlab
figure; margin(GH);
figure; nyquist(GH);
figure; rlocus(GH);
figure; step(Ls); grid on;
```

Edit parameters in `DCMotor.m` before rerunning the full script; values set manually in the workspace are overwritten by the script.

## Results reported in the study

For the selected tuning, the technical report lists:

| Metric | Reported value |
| --- | ---: |
| Rise time | 238.116 s |
| Settling time | 423.156 s |
| Overshoot | None reported |
| Steady-state tracking error | 0 rad |

These are results documented in the report, not a new execution performed for this README. The script calls `stepinfo(Ls)` with MATLAB's default response thresholds.

## Scope and limitations

This is an academic, continuous-time simulation study. Gravity is linearized around $\theta=0$, so the supplied 1 rad reference should be understood as a linear-model test rather than validation of large-angle physical behavior.

The controller uses an ideal derivative. The model does not include derivative filtering, voltage or current saturation, anti-windup, sensor noise, digital sampling or hardware validation. These effects would need to be addressed before practical implementation.
