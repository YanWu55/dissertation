import numpy as np
import pandas as pd
import json
import time
from scipy.integrate import solve_ivp
import scipy
import matplotlib.pyplot as plt
import sys
from pathlib import Path


# ============================================================
# Duffing Stability Benchmark Reference Generation
# ============================================================

OUTPUT_DIR = Path("Duffing_Stability_References")
OUTPUT_DIR.mkdir(exist_ok=True)


# ------------------------------------------------------------
# 1. Benchmark parameters
# ------------------------------------------------------------

delta = 0.2
alpha = -1.0
beta = 1.0

T_FINAL = 60.0

RTOL = 1e-10
ATOL = 1e-12
MAX_STEP = 0.01

# Recovery target:
# |q - 1| <= 0.02
# |v|     <= 0.02
Q_TOL = 0.02
V_TOL = 0.02

# Fixed sampling resolution used to determine settling time
SETTLING_DT = 0.001


# ------------------------------------------------------------
# 2. Duffing dynamics
#
# q_dot = v
# v_dot = -delta*v - alpha*q - beta*q^3
#
# For alpha = -1, beta = 1:
# v_dot = -0.2*v + q - q^3
# ------------------------------------------------------------

def duffing(t, z):
    q, v = z

    dq = v
    dv = -delta * v - alpha * q - beta * q**3

    return [dq, dv]


# ------------------------------------------------------------
# 3. Mechanical energy
#
# H(q,v) = 0.5*v^2 + 0.5*alpha*q^2
#          + 0.25*beta*q^4
#
# With alpha=-1, beta=1:
# H = 0.5*v^2 - 0.5*q^2 + 0.25*q^4
# ------------------------------------------------------------

def energy(q, v):
    return (
        0.5 * v**2
        + 0.5 * alpha * q**2
        + 0.25 * beta * q**4
    )


# ------------------------------------------------------------
# 4. Benchmark instances
# ------------------------------------------------------------

cases = [
    {
        "difficulty": "Baseline",
        "initial_state": [0.89443, 0.0],
        "target_energy": -0.24
    },
    {
        "difficulty": "Intermediate",
        "initial_state": [0.47477, 0.0],
        "target_energy": -0.10
    },
    {
        "difficulty": "Challenge",
        "initial_state": [0.14214, 0.0],
        "target_energy": -0.01
    }
]


# ------------------------------------------------------------
# 5. Settling-time definition
#
# First time after which BOTH
#
# |q - 1| <= 0.02
# |v|     <= 0.02
#
# remain satisfied until T_FINAL.
# ------------------------------------------------------------

def calculate_settling_time(solution):

    t_check = np.arange(
        0.0,
        T_FINAL + SETTLING_DT,
        SETTLING_DT
    )

    z = solution.sol(t_check)

    q = z[0]
    v = z[1]

    inside_target = (
        (np.abs(q - 1.0) <= Q_TOL)
        &
        (np.abs(v) <= V_TOL)
    )

    # If never permanently inside target
    if not inside_target[-1]:
        return np.nan

    # Find the final time at which the state is outside the target
    outside_indices = np.where(~inside_target)[0]

    if len(outside_indices) == 0:
        return 0.0

    last_outside = outside_indices[-1]

    if last_outside + 1 >= len(t_check):
        return np.nan

    return t_check[last_outside + 1]


# ------------------------------------------------------------
# 6. Run all benchmark instances
# ------------------------------------------------------------

results = []

trajectories = {}

for case in cases:

    difficulty = case["difficulty"]
    z0 = np.array(case["initial_state"], dtype=float)

    print("\n==========================================")
    print(f"Running Duffing Stability: {difficulty}")
    print("==========================================")

    H0 = energy(z0[0], z0[1])

    print(f"Initial state: q0 = {z0[0]:.5f}, v0 = {z0[1]:.5f}")
    print(f"Initial energy: H0 = {H0:.8f}")

    start_time = time.perf_counter()

    sol = solve_ivp(
        duffing,
        [0.0, T_FINAL],
        z0,
        method="DOP853",
        rtol=RTOL,
        atol=ATOL,
        max_step=MAX_STEP,
        dense_output=True
    )

    computation_time = time.perf_counter() - start_time

    settling_time = calculate_settling_time(sol)

    recovery_success = not np.isnan(settling_time)

    print(f"Solver success: {sol.success}")
    print(f"Recovery success: {recovery_success}")
    print(f"Settling time: {settling_time:.3f} s")
    print(f"Computation time: {computation_time:.3f} s")

    # Fixed output grid for stored reference trajectory
    t_store = np.arange(
        0.0,
        T_FINAL + SETTLING_DT,
        SETTLING_DT
    )

    z_store = sol.sol(t_store)

    trajectories[difficulty] = {
        "t": t_store,
        "q": z_store[0],
        "v": z_store[1]
    }

    # Save each reference trajectory
    trajectory_df = pd.DataFrame({
        "time_s": t_store,
        "q": z_store[0],
        "v": z_store[1]
    })

    trajectory_file = (
        OUTPUT_DIR /
        f"DUFF_STAB_{difficulty.upper()}_trajectory.csv"
    )

    trajectory_df.to_csv(
        trajectory_file,
        index=False
    )

    results.append({
        "Difficulty": difficulty,
        "q0": z0[0],
        "v0": z0[1],
        "Initial_Energy": H0,
        "Reference_Energy_Level": case["target_energy"],
        "Recovery_Success": recovery_success,
        "Settling_Time_s": settling_time,
        "Computation_Time_s": computation_time,
        "Solver_Success": sol.success
    })


# ------------------------------------------------------------
# 7. Save benchmark reference-results table
# ------------------------------------------------------------

results_df = pd.DataFrame(results)

results_file = (
    OUTPUT_DIR /
    "Duffing_Stability_reference_results.csv"
)

results_df.to_csv(
    results_file,
    index=False
)

print("\n==========================================")
print("REFERENCE RESULTS")
print("==========================================")

print(results_df.to_string(index=False))


# ------------------------------------------------------------
# 8. Save reproducibility metadata
# ------------------------------------------------------------

metadata = {
    "system": "Duffing Oscillator",
    "benchmark_family":
        "Multiple-Equilibrium Stability and Basin-Recovery Benchmark",

    "parameters": {
        "delta": delta,
        "alpha": alpha,
        "beta": beta
    },

    "solver": "SciPy solve_ivp DOP853",

    "scipy_version": scipy.__version__,
    "python_version": sys.version,

    "rtol": RTOL,
    "atol": ATOL,
    "max_step_s": MAX_STEP,

    "time_horizon_s": T_FINAL,

    "target_set": {
        "q_condition": "|q - 1| <= 0.02",
        "v_condition": "|v| <= 0.02"
    },

    "settling_time_definition":
        "First time after which the state remains inside "
        "the target set until the end of the 60 s horizon.",

    "settling_time_sampling_interval_s": SETTLING_DT,

    "reference_type":
        "Analytical + Numerical"
}

metadata_file = (
    OUTPUT_DIR /
    "Duffing_Stability_metadata.json"
)

with open(metadata_file, "w") as f:
    json.dump(metadata, f, indent=4)


# ------------------------------------------------------------
# 9. Plot trajectories
# ------------------------------------------------------------

plt.figure(figsize=(8, 6))

for case in cases:

    difficulty = case["difficulty"]

    q = trajectories[difficulty]["q"]
    v = trajectories[difficulty]["v"]

    plt.plot(
        q,
        v,
        label=difficulty
    )

# Stable equilibrium
plt.plot(
    1.0,
    0.0,
    "ko",
    label="Stable equilibrium (1,0)"
)

plt.xlabel(r"$q$")
plt.ylabel(r"$v$")
plt.legend()
plt.grid(True)

plt.tight_layout()

plt.savefig(
    OUTPUT_DIR /
    "Duffing_Stability_phase_trajectories.pdf"
)

plt.savefig(
    OUTPUT_DIR /
    "Duffing_Stability_phase_trajectories.png",
    dpi=300
)

plt.close()


# ------------------------------------------------------------
# 10. Plot distance to target versus time
# ------------------------------------------------------------

plt.figure(figsize=(8, 6))

for case in cases:

    difficulty = case["difficulty"]

    t = trajectories[difficulty]["t"]
    q = trajectories[difficulty]["q"]
    v = trajectories[difficulty]["v"]

    target_error = np.maximum(
        np.abs(q - 1.0) / Q_TOL,
        np.abs(v) / V_TOL
    )

    plt.plot(
        t,
        target_error,
        label=difficulty
    )

plt.axhline(
    1.0,
    linestyle="--",
    label="Target-set boundary"
)

plt.xlabel("Time [s]")
plt.ylabel("Normalised target error")

plt.legend()
plt.grid(True)

plt.tight_layout()

plt.savefig(
    OUTPUT_DIR /
    "Duffing_Stability_target_error.pdf"
)

plt.savefig(
    OUTPUT_DIR /
    "Duffing_Stability_target_error.png",
    dpi=300
)

plt.close()


print("\nGenerated files are stored in:")
print(OUTPUT_DIR.resolve())