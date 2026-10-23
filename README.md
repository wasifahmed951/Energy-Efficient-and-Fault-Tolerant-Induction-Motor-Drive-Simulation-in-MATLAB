# Energy-Efficient and Fault-Tolerant Induction Motor Drive — MATLAB/Simulink code

Code for the thesis *"Energy-Efficient and Fault-Tolerant Induction Motor Drive: A Comparative
Study of Metaheuristic-Tuned IFOC and DTC with Neural-Network Loss-Minimization"*, including
the AGV (mobile-robot) case study. The Python ML part comes later; this code exports the data it needs.

## 1. Requirements

| Needed | For |
|---|---|
| MATLAB R2021a or newer | everything |
| Simulink | the two models (`IM_DRIVE.slx`, `AGV_MISSION.slx`) |
| Optimization Toolbox | `fminbnd` (loss-minimising flux) |
| Global Optimization Toolbox | `ga`, `particleswarm` (if missing, the built-in `ga_basic` / `pso_basic` are used) |
| Parallel Computing Toolbox (optional) | `parsim` / `parfor` for the sweeps — much faster |

Simscape Electrical is **not** required: the motor and inverter are written as MATLAB code inside
MATLAB Function blocks. That was necessary because the thesis needs things the standard
Simscape machine block does not give directly (saturation-aware loss logging, an open phase
with an optional neutral connection, sensor faults).

## 2. First run (10 minutes)

```matlab
cd IM_Drive_Thesis
setup_project                      % adds all folders to the path
build_drive_model                  % creates simulink/IM_DRIVE.slx
build_agv_model                    % creates simulink/AGV_MISSION.slx
P = default_params('IFOC');
res = run_sim(P, make_scenario('tuning', P));       % Simulink
plot_run(res, 'IFOC check');
M = compute_metrics(res)
```

If the Simulink build or run gives an error, run the same thing with the plain-MATLAB engine
to separate a model problem from a code problem:

```matlab
res = run_sim(P, make_scenario('tuning', P), 'matlab');
```

Both engines call exactly the same functions (`ctrl_step`, `plant_step`, `agv_step`), so the
results are the same. Every function was tested with the `'matlab'` engine (see section 8);
the Simulink model builders could not be run where this code was written, so section 2 is the
first thing to check on your PC.

Then run `main_run_all.m` **section by section** (Ctrl+Enter), checking each result.

## 3. Folder layout

| Folder | What is inside |
|---|---|
| `params/` | `default_params.m` — every parameter (motor, inverter, controllers, faults, AGV) |
| `core/` | the model and controllers: `plant_step` (inverter + saturated motor, open-phase modes, sensor faults, losses), `ctrl_step` (IFOC, DTC, SVPWM, flux reference, current observer, MRAS, fault detection and reconfiguration) |
| `sim/` | `make_scenario` (test cycles), `run_sim` (Simulink or MATLAB engine), `run_batch` (parsim), `log_index` (log columns) |
| `simulink/` | `build_drive_model.m`, `build_agv_model.m` (create the .slx files), `add_mlfb.m` |
| `analysis/` | `compute_metrics` (all metrics), `plot_run` |
| `tuning/` | Ziegler–Nichols relay test, common cost, GA/PSO with equal budget, DTC band calibration |
| `efficiency/` | steady-state loss model, analytic optimal flux, look-up table, flux sweep (CSV for Python), efficiency comparison |
| `faults/` | fault study, labelled fault-window dataset for the Python classifiers |
| `agv/` | AGV path, kinematics, pure pursuit, mission and fault supervisor, mission metrics |
| `results/`, `dataset/` | outputs |
| `tests/` | the test scripts used during development (plain-MATLAB engine) |
| `blocks/` | block-diagram versions of both drives (`IFOC_BLOCKS.slx`, `DTC_BLOCKS.slx`) built from standard Simulink blocks: `build_block_models.m`, `block_model_params.m`, `run_block_demo.m` (section 10) |

## 4. How the code maps to the proposal

| Proposal | Code |
|---|---|
| 5.1 Motor and inverter model | `core/plant_step.m`, `core/im_currents.m`, `core/sat_Lms.m` |
| 5.2 IFOC, DTC, fairness rule | `core/ctrl_step.m` (`ifoc_inner`, `dtc_inner`), `tuning/calibrate_dtc_bands.m` |
| 5.3 ZN, GA, PSO, cost J | `tuning/run_zn_relay.m`, `tuning/tuning_cost.m`, `tuning/run_tuning.m` |
| 5.4 Simulink data → Python ML | `efficiency/run_flux_sweep.m` (writes `results/flux_sweep_*.csv`), `build_flux_lut.m`, MLP weights in `P.mlp` (flux mode 4) |
| 5.5 Faults + reconfiguration | `ctrl_step.m` (TDO, residuals, MRAS, observer, neutral remedy), `faults/run_fault_study.m` |
| 5.5 ML fault diagnosis data | `faults/export_fault_dataset.m` → `dataset/fault_windows.mat` |
| 5.6 Test cycles and metrics | `sim/make_scenario.m`, `analysis/compute_metrics.m` |
| 5.7 AGV case study | `agv/*`, `simulink/build_agv_model.m` |

## 5. Model summary

**Plant.** Two-level inverter with centre-aligned PWM (double update). The motor is the
alpha–beta flux model with saturation of the magnetising inductance,
Lms(|ψm|) = Lm0 / (1 + a·(|ψm|/ψk)^b). It is integrated with RK4 over every constant-voltage
PWM segment, so current and torque ripple are exact; logged powers use the trapezoid rule
(power balance closes to <0.1%). Losses: stator and rotor copper, core loss
(kh·f + ke·f²)·|ψm|², friction, IGBT/diode conduction and switching.

**Open phase a.** Mode 2: isolated neutral (i_alpha = 0, single-phase operation).
Mode 3: neutral connected to the DC mid-point (zero-sequence current allowed). In mode 3 the
alpha axis behaves like a winding with 3·Rs and 3·Lls driven by −(vb + vc), which is what the
IFOC remedy remaps its voltages to.

**Controllers.** IFOC: rotor-flux current model, slip calculation, dq PI current loops with
decoupling, SVPWM by min–max injection. DTC: voltage-model stator flux, 2-level flux and 3-level
torque hysteresis, classical six-sector table. Both: speed PI with anti-windup (or relay for ZN),
flux reference (rated / fixed / LUT / MLP) with rated-flux fallback during transients.

**Faults.** Speed encoder (reading → 0), phase-a current sensor (gain → 0), open phase a.
Detection: third-difference operator plus model residuals (observer for current, MRAS for speed).
A phase-a anomaly is classified by Kirchhoff's law on the b and c sensors: if ib + ic ≈ 0 the
phase is open, otherwise the sensor failed. Reconfiguration: MRAS speed, observer current,
neutral connection + voltage remapping (IFOC), torque derating.

## 6. Parameters to replace with your own

In `params/default_params.m`, marked PLACEHOLDER:

* saturation curve `sat_a`, `sat_b` — fit to a no-load test, or ask your supervisor for data;
* core-loss coefficients `kh`, `ke` (about 120 W at rated by default);
* AGV mass, payload, gear ratio, wheel size — take from a real AGV/tugger datasheet.

The motor's electrical parameters are the Simulink "4 kW, 400 V, 50 Hz, 1430 rpm" preset; the
plant reproduces the nameplate (1431 rpm at rated torque in an open-loop V/f test).

## 7. Outputs

| File | Content |
|---|---|
| `results/tuning_runs.mat`, `tuned_params.mat` | GA/PSO runs (convergence in `out.hist`), best gains |
| `results/comparison_rq1_rq2.mat` | metrics for both drives × ZN/GA/PSO × test cycles |
| `results/flux_sweep_IFOC.csv`, `..._DTC.csv` | one row per sweep run — **training data for Python** |
| `results/flux_opt_*.csv` | loss-minimising flux per operating point |
| `results/efficiency_*.csv` | rated vs LUT (vs MLP) flux at light load |
| `results/fault_study.csv` | detection, post-fault ripple, speed error, current |
| `dataset/fault_windows.mat` | X [n×200×6], y, meta — **training data for the classifiers** |
| `results/agv_results.mat` | AGV mission metrics |

Log columns: `I = log_index()` → `res.lp(:, I.p.Te)`, `res.lc(:, I.c.Te_ref)`, …
AGV log columns are listed at the top of `agv/agv_metrics.m`.

**Coming back from Python.** Either write a CSV `w,T,psi` on the `P.lut` grid and call
`P = build_flux_lut(P, 'csv', file)` (works for XGBoost, KAN or any model), or put the MLP weights
into `P.mlp` (W1 10×2, b1, W2 1×10, b2, and the input/output scaling) and set
`P.ctrl.flux_mode = 4`.

## 8. Tested results (plain-MATLAB engine, Octave 8.4)

| Check | Result |
|---|---|
| Open-loop V/f, rated load | 1430.9 rpm (nameplate 1430), 11.6 A peak, motor η 0.90 |
| Power balance | electrical input vs output + losses within 0.5 W of 1205 W |
| IFOC tuning cycle | speed, torque, flux and currents track; MRAS within ~1 rad/s; no false alarms |
| DTC tuning cycle | tracks; 3.0 kHz switching with default bands (calibrate to 5 kHz) |
| Speed-sensor fault | detected in 0.1–0.2 ms, speed held on MRAS (both drives) |
| Current-sensor fault | detected and classified in 4 ms, speed held (both drives) |
| Open phase, IFOC + neutral remedy | speed held (74.8 vs 74.9 rad/s), torque ripple 2.7 → 6.5 N m |
| Open phase without remedy | drive stalls at 50% load (IFOC and DTC) |
| Analytic optimum flux | 6–7% input-power saving at 25% load, <1% at 50% load |
| Closed-loop LUT flux (0.6 rated speed, 25% load) | input power 803 → 754 W (−6.1%), drive η 74.7% → 79.5% |
| DTC band calibration | 5.0 kHz average switching at h_T ≈ 1.05 N m, h_psi ≈ 0.007 Wb (now the defaults) |
| ZN relay test (IFOC) | Ku = 16.6, Pu = 4.5 ms → Kp_w = 7.46, Ki_w = 1993 |
| PSO / GA fallbacks | run with the evaluation budget respected and convergence history recorded |
| Fault dataset export | windows labelled correctly (healthy / speed / current / open) |
| AGV mission (short route, IFOC + LUT flux) | completed in 13.5 s, 0.67 Wh, max cross-track error 9.8 cm, no false alarms |
| Not yet run here | the Simulink models themselves (run `check_install.m` first) and the AGV open-phase mission |

## 9. Run times

Simulink compiles the MATLAB Function blocks to C, so it is much faster than the
plain-MATLAB loop. Expect roughly seconds per simulated second for one drive. The GA/PSO
stage (12,000 runs) is the longest: use `PARALLEL = true`, reduce `SEEDS` to `1:3`, or reduce
`BUDGET` while developing.

## 10. Block-diagram models (editable)

`IM_DRIVE.slx` and `AGV_MISSION.slx` hide the drive inside MATLAB Function blocks. `blocks/`
builds the same thesis models from standard Simulink blocks, so every equation is visible and
editable, and they give the same results as the thesis script:

```matlab
restoredefaultpath; setup_project
bdclose all; clear functions
build_block_models                 % builds and opens all four models
build_block_models('IFOC')         % or one of 'IFOC', 'DTC', 'AGV_IFOC', 'AGV_DTC'
```

| Model | What it is |
|---|---|
| `IFOC_BLOCKS.slx` | one IFOC drive (Ts 100 us, SVPWM 5 kHz) |
| `DTC_BLOCKS.slx` | one classical DTC drive (Ts 25 us) |
| `AGV_IFOC_BLOCKS.slx` | AGV mission (A to B, 3 s dwell, payload drop, back to A) with two IFOC wheel drives |
| `AGV_DTC_BLOCKS.slx` | the same mission with two DTC wheel drives |

**Same equations, same parameters.** `block_model_params.m` (runs before every simulation)
loads the thesis struct `P` from `results/tuned_params.mat`, exactly as `main_run_all` uses it.
The blocks follow the thesis code line by line:
- **Controller** (`core/ctrl_step.m`): discrete time at Ts. Every controller variable is a Fcn
  block `x = f(...)` and every controller state a Unit Delay `x(k-1)`. It has subsystems for
  model inductances, current reconstruction, MRAS speed estimator, fault detection and isolation,
  speed controller (PI or ZN relay), flux reference (rated / fixed / LUT / MLP with transient
  hold), IFOC current control and SVPWM or DTC observer and switching table, applied voltage and
  current observer.
- **Inverter and motor** (`core/plant_step.m`): centre-aligned double-update PWM with the thesis
  carrier phase, conduction and switching losses, saturated motor with the open-phase modes, core
  loss.
- **AGV** (`agv/agv_step.m`): kinematics, energy, path progress, supervisor, mission logic,
  pure pursuit, loads, fault injection.

**Settings on the top level** (double-click, then press Run):

| Block | Values |
|---|---|
| `Reference source` (drive models) | 1 manual blocks below, 2 thesis test cycle (`scenario` in `block_model_params.m`) |
| `Speed ref (rpm)`, `Load torque (Nm)`, fault switches (drive models) | manual test cycle; faults start at `t_fault` |
| `Flux select` | 1 rated, 2 fixed, 3 look-up table, 4 MLP (= `P.ctrl.flux_mode`) |
| `LUT source` | 1 analytic loss model (thesis `P.lut`), 2 ML table (KAN for IFOC, MLP for DTC) |
| `Speed controller` | 1 PI, 2 Ziegler-Nichols relay test |
| `Fault type`, `Fault side` (AGV models) | 0 none, 1 speed sensor, 2 current sensor, 3 open phase; 1 left, 2 right |

**Check against the thesis script:**
```matlab
compare_block_vs_script('IFOC')                                   % rated speed step
compare_block_vs_script('DTC', 'fault', struct('fault', 'open'))  % any make_scenario cycle
compare_block_vs_script('AGV_IFOC', 3, 0)                         % AGV: flux_mode 3, healthy
```
It runs the block model and the thesis script on the same cycle and prints both sets of metrics
(`compute_metrics.m` for both). Checked before release (17 Oct 2026, after the two fixes below):
- **Drive metrics:** every metric within 0.1 %. The exception is IFOC THD50 at about 0.7 %; its value is
  only 0.07 %, so the difference is 0.0005 percentage points, the numerical noise of the two
  integrators.
- **Detection times:** identical.
- **AGV:** mission time, energy and path error within 0.05 %.

**Fixes in the thesis code (17 Oct 2026, found with the block models):**
- **Time stamps:** `sim/unpack_sim_output.m` now stamps the Simulink logs k*Ts like the plain-MATLAB engine.
  Before, the open-phase "pre-fault" ripple included the first faulty step, and detection times were
  0.1 ms short.
- **Mean squares:** `core/plant_step.m` now integrates Te^2 and ia^2 exactly between switching instants.
  The old trapezoid gave rms torque ripple about 13 % (IFOC) and 12-17 % (DTC) too high, and total
  THD too high.
- **Affected results:** `rerun_after_fix.m` re-runs only the affected results (sections 5-7, fairness
  checks, section 10 A/B) and `tuning/rescore_tuning.m` re-scores the tuning cost J. The GA/PSO
  searches, tuned gains, AGV missions, flux sweep and Python results are not affected.

The only modelling difference is numerical: the block models integrate the motor with fixed-step
RK4 (`h_step` = 5 us drive, 25 us AGV; the comparison uses 1 us for IFOC) instead of integrating
exactly between switching instants. Report thesis numbers from the scripts; use the block models
to explain, draw figures and experiment. An AGV run takes several minutes.
