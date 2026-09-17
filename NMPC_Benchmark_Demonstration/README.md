# NMPC Benchmark Demonstration: Cart-Pole + CSTR

This MATLAB project implements the Chapter 5 tutorial demonstration:

- **One algorithm:** Nonlinear Model Predictive Control (NMPC)
- **Benchmark Case I:** Cart-Pole (`CP-1`)
- **Benchmark Case II:** Continuous Stirred-Tank Reactor (`CSTR-1`)
- **Common reporting:** regulation error, settling time, control effort, constraint violations, closed-loop cost, solver time, solver success rate

## Requirements

- MATLAB
- Model Predictive Control Toolbox
- Optimization Toolbox

No Simulink model is required. The demonstration runs entirely from MATLAB `.m` files.

## How to run

1. Extract this folder.
2. Open MATLAB and set this folder as the current folder.
3. Run:

```matlab
run_all
```

The script:
- checks the required toolboxes;
- validates both NMPC prediction models;
- runs both closed-loop simulations;
- saves figures to `results/`;
- saves individual benchmark metrics as CSV;
- saves a cross-system comparison table as CSV;
- saves complete MATLAB result structures as MAT files.

You can also run each benchmark separately:

```matlab
cp = run_cartpole_demo;
cstr = run_cstr_demo;
```

## Benchmark definitions used in this code

### CP-1

State order:

```text
x = [cart position; pole angle; cart velocity; pole angular velocity]
```

Target:

```text
x* = [0; 0; 0; 0]
```

Default initial condition:

```text
x0 = [0.20; 0.15; 0; 0]
```

The pole angle is measured from the **upright** configuration, so `theta = 0` is the unstable upright equilibrium.

Default plant parameters:

- Cart mass `M = 1 kg`
- Pole mass `m = 1 kg`
- COM distance `l = 0.5 m`
- Gravity `g = 9.81 m/s^2`
- Cart viscous damping `Kd = 10 N s/m`

Constraints:

- Cart position: `-2.4 <= z <= 2.4 m`
- Force: `-100 <= F <= 100 N`

NMPC defaults:

- `Ts = 0.1 s`
- Prediction horizon = 20
- Control horizon = 5

This is a **local upright regulation benchmark**, not a full swing-up benchmark.

### CSTR-1

State order:

```text
x = [reactant concentration C_A; reactor temperature T]
```

Manipulated input:

```text
u = coolant temperature T_c
```

The nonlinear model is the standard two-state non-isothermal CSTR:

```text
dCA/dt = F/V (CAf - CA) - r
dT/dt  = F/V (Tf - T) - ΔH/(ρCp) r - UA/(ρCp V)(T - Tc)

r = k0 exp(-E/(R T)) CA
```

Parameters are the standard values used in the MathWorks CSTR model documentation:

- `F = 1 m^3/h`
- `V = 1 m^3`
- `R = 1.985875 kcal/(kmol K)`
- `ΔH = -5960 kcal/kmol`
- `E = 11843 kcal/kmol`
- `k0 = 34930800 1/h`
- `ρCp = 500 kcal/(m^3 K)`
- `UA = 150 kcal/(K h)`
- `CAf = 10 kmol/m^3`
- `Tf = 300 K`

Initial low-conversion operating point:

```text
x0 = [8.5698; 311.2639]
u0 = 292 K
```

The high-conversion target is defined by `CA* = 2 kmol/m^3`.
The code computes the **exact temperature and coolant temperature consistent with the nonlinear equations** rather than using only the rounded literature values near `T = 373 K`, `Tc = 299 K`.

Constraints:

- `0 <= CA <= 10 kmol/m^3`
- `300 <= T <= 390 K`
- `273 <= Tc <= 322 K`
- coolant move-rate: `-5 <= ΔTc <= 5 K/sample`

NMPC defaults:

- `Ts = 0.5 h`
- Prediction horizon = 12
- Control horizon = 6

## Important dissertation note

The closed-loop simulations check constraint satisfaction for the simulated initial condition. They are **not formal verification proofs for an uncertain initial set**.

Use wording such as:

> No constraint violation was observed in the closed-loop benchmark simulation.

Do not write:

> The system was formally verified safe.

unless you separately apply a formal verification method.

## If the solver struggles

NMPC is a nonlinear local optimisation method. If a run produces negative `ExitFlag` values:

1. Run `validateFcns` again (the setup functions do this automatically).
2. Reduce the initial displacement.
3. Increase the prediction horizon.
4. Adjust the output weights.
5. Temporarily soften/remove a state constraint.
6. Check the state/output scale factors.
7. Inspect `ExitFlag`, `Iterations`, and `PredictedCost` saved in the result structure.

All final dissertation results should be generated using one fixed configuration per benchmark and reported exactly.
