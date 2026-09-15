import numpy as np
import pandas as pd
import json
import scipy
from scipy.integrate import quad, simpson
from pathlib import Path
import matplotlib.pyplot as plt


# ============================================================
# Duffing Invariant-Set Reference Area Generation
# ============================================================

OUTPUT_DIR = Path("Duffing_Invariant_References")
OUTPUT_DIR.mkdir(exist_ok=True)


# ------------------------------------------------------------
# 1. Energy function
#
# H(q,v) = 0.5*v^2 - 0.5*q^2 + 0.25*q^4
#
# Potential:
# V(q) = -0.5*q^2 + 0.25*q^4
# ------------------------------------------------------------

def potential(q):
    return -0.5 * q**2 + 0.25 * q**4


# ------------------------------------------------------------
# 2. Benchmark energy levels
# ------------------------------------------------------------

cases = [
    {"difficulty": "Baseline",     "rho": -0.24},
    {"difficulty": "Intermediate", "rho": -0.10},
    {"difficulty": "Challenge",    "rho": -0.01},
]


# ------------------------------------------------------------
# 3. Numerical integration settings
# ------------------------------------------------------------

EPSABS = 1e-12
EPSREL = 1e-12

# Independent Simpson check
N_SIMPSON = 200001


# ------------------------------------------------------------
# 4. Compute reference geometry and areas
# ------------------------------------------------------------

results = []

for case in cases:

    difficulty = case["difficulty"]
    rho = case["rho"]

    print("\n==========================================")
    print(f"Running Duffing Invariant Set: {difficulty}")
    print("==========================================")

    # Solve V(q) = rho analytically.
    #
    # q^4 - 2q^2 - 4rho = 0
    # q^2 = 1 ± sqrt(1 + 4rho)

    s = np.sqrt(1.0 + 4.0 * rho)

    q_min = np.sqrt(1.0 - s)
    q_max = np.sqrt(1.0 + s)

    # Maximum |v| occurs at q = ±1
    v_max_at_well = np.sqrt(
        2.0 * (rho - potential(1.0))
    )

    # Velocity boundary for a given q:
    #
    # 0.5*v^2 + V(q) = rho
    #
    # |v| = sqrt(2*(rho - V(q)))

    def v_boundary(q):
        value = 2.0 * (rho - potential(q))
        return np.sqrt(max(0.0, value))

    # Cross-sectional height in velocity direction:
    #
    # v_upper - v_lower = 2*v_boundary(q)

    def area_integrand(q):
        return 2.0 * v_boundary(q)

    # Area of the positive-q component
    positive_area, quad_error = quad(
        area_integrand,
        q_min,
        q_max,
        epsabs=EPSABS,
        epsrel=EPSREL,
        limit=500
    )

    # Full set has two symmetric components
    full_area = 2.0 * positive_area


    # --------------------------------------------------------
    # 5. Independent Simpson-rule consistency check
    # --------------------------------------------------------

    q_grid = np.linspace(
        q_min,
        q_max,
        N_SIMPSON
    )

    v_grid = np.sqrt(
        np.maximum(
            0.0,
            2.0 * (rho - potential(q_grid))
        )
    )

    # 2*v_grid = vertical width of one component
    # multiply by 2 again for the symmetric left component
    simpson_area = 2.0 * simpson(
        2.0 * v_grid,
        x=q_grid
    )

    validation_difference = abs(
        full_area - simpson_area
    )


    # --------------------------------------------------------
    # 6. Print results
    # --------------------------------------------------------

    print(f"rho                  = {rho:.5f}")
    print(f"Positive-well q_min  = {q_min:.10f}")
    print(f"Positive-well q_max  = {q_max:.10f}")
    print(f"Max |v| at q = ±1    = {v_max_at_well:.10f}")
    print(f"Positive-well area   = {positive_area:.10f}")
    print(f"Full two-well area   = {full_area:.10f}")
    print(f"Simpson check area   = {simpson_area:.10f}")
    print(
        f"Validation difference = "
        f"{validation_difference:.3e}"
    )


    results.append({
        "Difficulty": difficulty,
        "rho": rho,
        "Positive_Well_q_Min": q_min,
        "Positive_Well_q_Max": q_max,
        "Max_abs_v_at_q_pm1": v_max_at_well,
        "Positive_Well_Area": positive_area,
        "Full_Two_Component_Area": full_area,
        "Quad_Estimated_Error": quad_error,
        "Simpson_Check_Area": simpson_area,
        "Quad_Simpson_Difference":
            validation_difference
    })


# ------------------------------------------------------------
# 7. Save results table
# ------------------------------------------------------------

results_df = pd.DataFrame(results)

results_file = (
    OUTPUT_DIR /
    "Duffing_Invariant_reference_areas.csv"
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
        "Multi-Well Invariant-Set and Basin-Geometry Benchmark",

    "energy_function":
        "H(q,v) = 0.5*v^2 - 0.5*q^2 + 0.25*q^4",

    "energy_levels": {
        "Baseline": -0.24,
        "Intermediate": -0.10,
        "Challenge": -0.01
    },

    "numerical_method":
        "SciPy adaptive quadrature scipy.integrate.quad",

    "independent_check":
        "Composite Simpson integration",

    "epsabs": EPSABS,
    "epsrel": EPSREL,

    "simpson_grid_points": N_SIMPSON,

    "scipy_version": scipy.__version__,

    "reference_type":
        "Analytical + Numerical",

    "reference_description":
        "Analytical boundary geometry; "
        "numerical evaluation of set area."
}

metadata_file = (
    OUTPUT_DIR /
    "Duffing_Invariant_metadata.json"
)

with open(metadata_file, "w") as f:
    json.dump(
        metadata,
        f,
        indent=4
    )


# ------------------------------------------------------------
# 9. Plot the three reference invariant sets
# ------------------------------------------------------------

fig, axes = plt.subplots(
    1,
    3,
    figsize=(12, 4)
)

for ax, case in zip(axes, cases):

    difficulty = case["difficulty"]
    rho = case["rho"]

    s = np.sqrt(1.0 + 4.0 * rho)

    q_min = np.sqrt(1.0 - s)
    q_max = np.sqrt(1.0 + s)

    q_pos = np.linspace(
        q_min,
        q_max,
        2000
    )

    v_pos = np.sqrt(
        np.maximum(
            0.0,
            2.0 * (rho - potential(q_pos))
        )
    )

    # Positive well
    ax.fill_between(
        q_pos,
        -v_pos,
        v_pos,
        alpha=0.5
    )

    # Negative well, by symmetry
    q_neg = -q_pos[::-1]
    v_neg = v_pos[::-1]

    ax.fill_between(
        q_neg,
        -v_neg,
        v_neg,
        alpha=0.5
    )

    ax.set_title(difficulty)
    ax.set_xlabel("q")
    ax.set_ylabel("v")
    ax.grid(True)

fig.tight_layout()

fig.savefig(
    OUTPUT_DIR /
    "Duffing_Invariant_reference_sets.pdf"
)

fig.savefig(
    OUTPUT_DIR /
    "Duffing_Invariant_reference_sets.png",
    dpi=300
)

plt.close(fig)


print("\nGenerated files are stored in:")
print(OUTPUT_DIR.resolve())
