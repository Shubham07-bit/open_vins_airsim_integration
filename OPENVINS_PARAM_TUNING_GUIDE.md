# OpenVINS Parameter Tuning Guide (AirSim Integration)

This guide explains every parameter in `estimator_config.yaml`, how to tune
them for the AirSim+ArduPilot+OpenVINS setup, and what physical/algorithmic
constraints govern monocular VIO performance at altitude.

It is written specifically for **monocular, downward/forward-tilted camera,
high-altitude flight** scenarios. Tuning recommendations are biased toward that
use case, not for handheld VIO datasets.

---

## Table of contents

1. [The fundamental constraint](#1-the-fundamental-constraint)
2. [Files and how they get loaded](#2-files-and-how-they-get-loaded)
3. [Parameter reference](#3-parameter-reference) — every param, ranked by importance
4. [Tuning workflows](#4-tuning-workflows-for-failure-modes) — recipes per failure mode
5. [The chi-square gating trap](#5-the-chi-square-gating-trap) — why drift cannot recover
6. [Operational procedures](#6-operational-procedures) — startup, recovery, debugging

---

## 1. The fundamental constraint

Most tuning decisions trace back to this single equation:

```
disparity_per_frame  =  (f * v_perp * dt) / D     [pixels]
```

| Symbol | Meaning |
|---|---|
| `f` | focal length in pixels (≈ image_width / (2·tan(HFOV/2))) |
| `v_perp` | translational velocity perpendicular to optical axis (m/s) |
| `dt` | frame interval in seconds (1/camera_rate_Hz) |
| `D` | depth to feature (m) |

OpenVINS' KLT tracker has a sub-pixel accuracy floor of about **0.1 px**. For
a feature to consistently pass the chi-square gate (with default
`up_msckf_sigma_px=1`, `chi2_multipler=1`), inter-frame disparity should be at
least **3–5× that**, i.e. ≥0.5 px/frame.

**Practical altitude ceiling**:

```
H_max  ≈  (f * v_perp * dt) / 0.5
```

| Resolution | f at 62° HFOV | Ceiling at v=10 m/s, 10 Hz | Ceiling at v=15 m/s, 10 Hz |
|---|---|---|---|
| 640×480 | 533 | ~120–180 m | ~180–270 m |
| 960×720 | 800 | ~180–270 m | ~270–400 m |
| 1280×960 | 1065 | ~240–360 m | ~360–540 m |
| 1456×1088 (RPi GS native) | 1209 | ~270–410 m | ~400–600 m |

These are upper bounds. Real ceilings are 50–70% of these because of texture
quality, depth distribution across the image, and the chi-square trap (§5).

**The four levers that move the ceiling**, in order of effectiveness:
1. **Resolution** — doubles `f` and the ceiling for ~4× CPU cost.
2. **Cruise speed** — linear gain in `v_perp`. Hover is the worst case.
3. **Camera angle** — −30° to −45° pitch maximizes useful image disparity.
4. **Loosened chi-square gates** — accepts marginal features at the cost of
   tolerating outliers.

---

## 2. Files and how they get loaded

The config is split across three files in **two locations** (source AND install).
**The launch loads the install copies**, not the source. Always verify edits
land in the install copy.

```
SOURCE (edit here for git):
  /mnt/rnd/Shubham/openvins_ws/open_vins/config/airsim_vio/
    estimator_config.yaml
    kalibr_imucam_chain.yaml
    kalibr_imu_chain.yaml

INSTALL (loaded by launch — must mirror source):
  /mnt/rnd/Shubham/openvins_ws/install/ov_msckf/share/ov_msckf/config/airsim_vio/
    estimator_config.yaml
    kalibr_imucam_chain.yaml
    kalibr_imu_chain.yaml
```

After editing source, run:
```
colcon build --symlink-install --packages-select ov_msckf
```
or just edit the install copy directly for short-term tests.

The launch file `airsim_vio.launch.py` resolves the config via
`ament_index_python.get_package_share_directory("ov_msckf")` → install copy.

---

## 3. Parameter reference

Every parameter is tagged:
- **TUNE** — directly affects altitude/drift behavior. Tune for your scene.
- **USEFUL** — matters in specific situations. Knob worth knowing.
- **LEAVE** — defaults are correct for AirSim. Don't touch.

### 3.1 Core algorithm

| Param | Default | Tag | Description |
|---|---|---|---|
| `verbosity` | INFO | LEAVE | Log level: ALL/DEBUG/INFO/WARNING/ERROR/SILENT. Override at launch with `verbosity:=DEBUG` for debugging. |
| `use_fej` | true | LEAVE | First-Estimate Jacobians for observability consistency. Always on. |
| `integration` | rk4 | LEAVE | IMU integration method. `rk4` is more accurate; `analytical` is faster. |
| `use_stereo` | false | LEAVE | Mono setup. |
| `max_cameras` | 1 | LEAVE | Mono setup. |
| `gravity_mag` | 9.81 | LEAVE | Earth gravity. |

### 3.2 Calibration switches

| Param | Default | Tag | Description |
|---|---|---|---|
| `calib_cam_extrinsics` | **false** | TUNE | Online calibrate camera-IMU transform. **MUST be false for AirSim** — online cal converges to a wrong attractor and corrupts the filter. The configured extrinsics from AirSim's `settings.json` are exact. |
| `calib_cam_intrinsics` | false | LEAVE | Online intrinsics cal. We know the exact f/cx/cy from FOV. |
| `calib_cam_timeoffset` | true | USEFUL | Online camera-IMU sync offset. Useful since the bridge runs IMU and camera in separate threads — small drift is normal. Keep on. |
| `calib_imu_intrinsics` | false | LEAVE | Sim IMU is perfect. |
| `calib_imu_g_sensitivity` | false | LEAVE | Sim IMU is perfect. |

### 3.3 State management (sliding window sizes)

| Param | Default | Tag | Description |
|---|---|---|---|
| `max_clones` | 11 | TUNE | Number of past camera poses kept in the sliding window. Bigger window = longer feature lifetimes = better triangulation of distant features. **Cost: O(n²) compute and memory**. Bump to 15 for high-altitude work where features need long baselines to triangulate. |
| `max_slam` | 75 | USEFUL | Max number of long-lived SLAM features kept in state. More = stronger constraints. Diminishing returns above 50. |
| `max_slam_in_update` | 25 | USEFUL | Subset of `max_slam` actually used per update. |
| `max_msckf_in_update` | 40 | TUNE | Max ephemeral (MSCKF) features used per update step. Higher = more constraints per update. **Bump to 50 if vision is borderline at altitude**. |
| `dt_slam_delay` | 1 | LEAVE | Seconds before MSCKF features get promoted to SLAM. Keep low (1s) for fast feature reuse. |

### 3.4 Feature representation

| Param | Default | Tag | Description |
|---|---|---|---|
| `feat_rep_msckf` | GLOBAL_3D | LEAVE | How MSCKF features are parameterized in state. GLOBAL_3D works fine. ANCHORED_INVERSE_DEPTH is also valid for distant features. |
| `feat_rep_slam` | ANCHORED_MSCKF_INVERSE_DEPTH | LEAVE | The right choice for distant features (better numerical stability than GLOBAL_3D when depth is large). |
| `feat_rep_aruco` | ANCHORED_MSCKF_INVERSE_DEPTH | LEAVE | ArUco unused. |

### 3.5 Zero-velocity update (ZUPT)

ZUPT injects a "velocity is zero" pseudo-measurement when the drone is judged
stationary. Critical for clean takeoff init; **dangerous in flight** if it
fires falsely.

| Param | Default | Tag | Description |
|---|---|---|---|
| `try_zupt` | true | USEFUL | Enable ZUPT updater at all. Keep on. |
| `zupt_chi2_multipler` | 1 | LEAVE | Chi-square gate for ZUPT itself. Keep TIGHT (1). False ZUPT firings during real motion are catastrophic. |
| `zupt_max_velocity` | 0.5 | LEAVE | Max state velocity (m/s) at which ZUPT is even considered. Default fine. |
| `zupt_noise_multiplier` | 50 | LEAVE | Inflation factor on ZUPT measurement noise — high value = ZUPT corrects gently rather than snapping the state. |
| `zupt_max_disparity` | 1.5 | TUNE | Max image disparity (px) to consider drone "visually stationary". **Scales with focal length** — at 1280×960 (f=1065) the same physical motion produces ~2× the pixel disparity as at 640×480 (f=533). **Bump to 3.0 for 1280×960**, otherwise hover ZUPT will fail to fire. |
| `zupt_only_at_beginning` | true | LEAVE | If true, ZUPT only fires before the first real motion is detected. **MUST be true** to prevent in-flight ZUPT killing real motion at altitude (where parallax is naturally low and looks like "stationary"). |

### 3.6 Initialization

| Param | Default | Tag | Description |
|---|---|---|---|
| `init_window_time` | 1.5 | LEAVE | Seconds of IMU/camera history collected for init. |
| `init_imu_thresh` | 0.5 | USEFUL | Min accel disturbance (m/s²) to declare "moved enough to start dynamic init". Lower = init triggers from rotor spin-up alone. Higher = needs actual takeoff motion. Default works for ArduCopter takeoffs. |
| `init_max_disparity` | 5.0 | LEAVE | Max image disparity allowed during static init phase. |
| `init_max_features` | 50 | LEAVE | Number of features used during dynamic init MLE. |
| `init_dyn_use` | true | LEAVE | Enable dynamic init (vs static-only). Keep on. |
| `init_dyn_mle_opt_calib` | false | LEAVE | Don't optimize extrinsics during init. Same reasoning as `calib_cam_extrinsics: false`. |
| `init_dyn_mle_max_iter` | 50 | LEAVE | MLE solver iterations cap. |
| `init_dyn_mle_max_time` | 0.5 | LEAVE | MLE solver time budget. |
| `init_dyn_mle_max_threads` | 6 | LEAVE | MLE thread count. Match CPU cores. |
| `init_dyn_num_pose` | 6 | LEAVE | Number of frames in MLE init window. |
| `init_dyn_min_deg` | 10.0 | LEAVE | Min parallax angle for init. Lower = inits faster but less accurate depth. |
| `init_dyn_inflation_*` | varies | LEAVE | Initial covariance inflation factors. Defaults are tuned. |
| `init_dyn_min_rec_cond` | 1e-15 | LEAVE | Min reciprocal condition number for MLE matrix. Defaults are fine. |
| `init_dyn_bias_g` | [0,0,0] | LEAVE | Prior on gyro bias. Sim IMU is perfect → zeros are correct. |
| `init_dyn_bias_a` | [0,0,0] | LEAVE | Prior on accel bias. Same. |

### 3.7 Feature tracking (KLT)

This is where high-altitude tuning happens.

| Param | Default | Tag | Description |
|---|---|---|---|
| `use_klt` | true | LEAVE | Use KLT tracker (vs descriptor-based). KLT is the right tracker for OpenVINS' MSCKF backend. |
| `num_pts` | 400 | TUNE | **Target** number of features per frame. The actual count depends on what FAST detects. **Bump to 600 for high-altitude scenes** with sparse texture — gives more candidates so more survive the chi-square gate. Cost: ~25% more CPU on KLT. |
| `fast_threshold` | 20 | TUNE | FAST corner detector intensity threshold. Lower = detect more (weaker) corners. **Bump down to 12–15 for high altitude** where texture is muted. Too low (<10) starts detecting noise as corners. |
| `grid_x`, `grid_y` | 8, 6 | TUNE | Spatial grid for distributing features across the image. At 1280×960, the cells are 160×160 px which is fine. **For 1280×960, consider 12, 8** (cells of ~107×120 px) to force features to spread across the wider image. Better observability. |
| `min_px_dist` | 10 | USEFUL | Minimum pixel distance between two tracked features. Forces spatial sparsity. Two features within 5 px see nearly the same 3D point, so this prevents redundancy and KLT template overlap. **Bump to 15** for distant ground scenes where features should be spread wider. Smaller (5) only useful for tiny textured regions. |
| `knn_ratio` | 0.70 | LEAVE | Lowe's ratio for descriptor matching. **Only used if `use_klt: false`**. Ignore. |
| `track_frequency` | 31.0 | LEAVE | Max tracker rate (Hz). Caps to actual camera rate. Set near or slightly above your real camera Hz. |
| `downsample_cameras` | false | LEAVE | If true, halves resolution before tracking. We just doubled it — leave false. |
| `num_opencv_threads` | 4 | LEAVE | OpenCV thread pool size. Match CPU cores. |
| `histogram_method` | CLAHE | USEFUL | Image preprocessing: CLAHE / HISTOGRAM / NONE. **Keep CLAHE for synthetic scenes** — AirSim sometimes has flat-lit areas. NONE is faster but worse for low-contrast. |

### 3.8 ArUco

Disabled. Ignore the four `*_aruco*` params.

### 3.9 Update noise / chi-square gating

This is the **most important** group for the high-altitude failure mode.

| Param | Default | Tag | Description |
|---|---|---|---|
| `up_msckf_sigma_px` | 2 | TUNE | Assumed pixel noise for MSCKF features. **Larger = filter trusts vision less, accepts wider residuals**. Was 1, loosened to 2 for high-altitude work. Push to 3 if features still rejected. |
| `up_msckf_chi2_multipler` | 3 | TUNE | Multiplier on the chi-square gating threshold. Was 1, loosened to 3. Push to 5–10 if features still rejected at altitude — but risk accepting outliers. |
| `up_slam_sigma_px` | 2 | TUNE | Same, for long-lived SLAM features. Match MSCKF setting. |
| `up_slam_chi2_multipler` | 3 | TUNE | Same. SLAM features are harder to keep — they need to pass the gate every frame for many frames. |
| `up_aruco_sigma_px` | 1 | LEAVE | ArUco disabled. |
| `up_aruco_chi2_multipler` | 1 | LEAVE | ArUco disabled. |

### 3.10 Recording and misc

| Param | Tag | Description |
|---|---|---|
| `record_timing_information` | LEAVE | Output timing CSV. Performance only. |
| `save_total_state` | LEAVE | Whether to write the trajectory log. Keep on. |
| `filepath_est`/`std`/`gt` | LEAVE | Output paths only. |
| `use_mask` | LEAVE | Image masking, unused. |
| `relative_config_imu` | LEAVE | Path to IMU calibration sub-file. |
| `relative_config_imucam` | LEAVE | Path to camera-IMU calibration sub-file. |

---

## 4. Tuning workflows for failure modes

Match your symptom to a workflow. Apply changes one at a time, re-test, look at
`/tmp/ov_debug.log` after each change.

### A. "ZUPT firing during flight" (in-flight `[ZUPT]: passed disparity`)

The drone is moving but ZUPT thinks it's stationary. Filter velocity is being
clamped to zero → drone "stops" in the estimate.

**Fix**:
1. `zupt_only_at_beginning: true` — most important.
2. Verify the launch loaded the correct config (grep `/tmp/ov_debug.log` for
   `zupt_only_at_beginning`).

### B. "Drift starts immediately after takeoff" (`MSCKF update (0 feats)` from frame 1)

Vision is being rejected from the very start. Either:
- Wrong T_imu_cam (extrinsic mismatch) → predicted feature locations are off
- Wrong intrinsics (resolution/FOV mismatch in config vs AirSim)
- `calib_cam_extrinsics: true` → online cal corrupting the filter

**Fix**:
1. Check `calib_cam_extrinsics: false`.
2. Verify `kalibr_imucam_chain.yaml` resolution and intrinsics match AirSim's
   actual resolution and FOV.
3. Verify `T_imu_cam` matches AirSim's `Pitch` setting (matrix recipes below).

### C. "Drift starts at altitude X meters" (works low, fails high)

Disparity-per-frame falls below the gate threshold as depth grows.

**Fix**, in this order:
1. `up_msckf_sigma_px: 3`, `up_msckf_chi2_multipler: 5` — looser gate.
2. `num_pts: 600`, `fast_threshold: 12` — more candidates.
3. **Increase resolution** in `settings.json` and `kalibr_imucam_chain.yaml`
   — biggest win, doubles ceiling.
4. Pitch the camera to **−30° to −45°** if currently more vertical.

### D. "Drift during pure vertical climb" (works in cruise, fails climbing)

Looming-only flow. Pure ascent with downward camera = degenerate monocular
case. See §1.

**Fix**:
1. Tilt camera back from −90° to **−45°**.
2. If already tilted, climb on a slope (not straight up) so there's lateral
   image motion.
3. Operationally: don't hover at extreme altitude — keep moving so disparity
   stays nonzero.

### E. "Drift during landing" (everything fine until descent)

Same as D in reverse — descent looks like "image zoom in" which is geometrically
identical to forward motion along optical axis. Worst case.

**Fix**:
1. Same tilt fix as D.
2. Stop trusting VIO output below 5 m altitude.
3. Or: hand off to ArduPilot's GPS+EKF for the landing phase.

### F. "Filter diverged, won't recover even when descending to good texture"

This is fundamental — see §5. **Recovery is not possible mid-flight**.

**Fix**:
1. RTL + land + restart OpenVINS from clean ground state.
2. Prevent the next divergence with workflows A–E.

---

## 5. The chi-square gating trap

**Why VIO never recovers from divergence** — read this before tuning anything.

The gate decides whether to accept a feature:

```
chi2 = (z_obs - z_pred)^T * S^(-1) * (z_obs - z_pred)
       where S = H * P * H^T + R
```

- `z_pred` = predicted pixel from current state estimate
- `z_obs` = observed pixel from KLT
- `P` = state covariance (uncertainty)
- `R` = pixel noise = `up_msckf_sigma_px^2`

Accept if `chi2 < threshold * up_msckf_chi2_multipler`.

### What happens once the filter drifts

Suppose the state estimate has drifted by 50 m horizontally. Then for every new
feature:

1. `z_pred` (where the filter thinks the feature should land) is wrong by
   tens-to-hundreds of pixels.
2. `(z_obs - z_pred)` is huge — say 200 px instead of <1 px.
3. `P` only grew from process noise during the divergence — not from
   corrected updates. So `S` is small (a few px²).
4. `chi2 = 200² / few_px² = thousands`. Threshold is ~6 (for 2 DoF, 95%
   chi-square). Gate **rejects**.
5. Every feature from the new frame is rejected for the same reason.
6. Pure-IMU step. Drift grows. Residuals get bigger. Rejection gets harder.

This is a **monotonic trap**: the further you've drifted, the *less* likely
any future feature is to be accepted. You cannot widen `chi2_multipler` enough
— even multiplier=1000 wouldn't help once residuals are at 200 px.

### Why coming back to good texture doesn't help

Standard MSCKF/EKF-VIO **has no place memory**. SLAM features only persist
~25–75 frames. There is no global feature database, no descriptor matching,
no loop closure. When you descend back over the takeoff zone, the tracker
sees "new" KLT features. Their predicted positions are computed using the
(wrong) drifted state, so they get rejected just like the high-altitude
features did. There's no mechanism for "wait, this looks like the lawn I took
off from."

### The bias trap on top

The IMU bias estimates `bg`, `ba` are part of the state. Once a few wrong
updates happen (or pure dead-reckoning runs with no corrections), the biases
drift to wrong values. After that, IMU integration is **systematically wrong
even if no time passes**:

- Wrong `ba` of 0.05 m/s² → 1 m drift in 6 s, 25 m in 30 s
- The only thing that can correct bias is good vision updates
- But vision is being rejected because position is wrong
- Position is wrong because bias was bad
- → Catch-22

### Implications

**Three real strategies**:
1. **Prevent the first divergence** (everything in §4 is aimed at this).
2. **Detect divergence and reset the filter** procedurally — kill OpenVINS,
   land, restart. Algorithmic recovery is not possible.
3. **Add an absolute reference** — loop closure (`ov_secondary`), GPS fusion,
   stereo camera. None of these are in your current build.

---

## 6. Operational procedures

### 6.1 Startup order

1. **AirSim** — must be running first, with `settings.json` loaded.
2. **ArduPilot SITL** — connects to AirSim's TCP port.
3. **Bridge** — `python3 airsim_openvins_bridge.py` — must connect to AirSim
   AND publish `/imu`, `/camera/image`, `/camera/camera_info`.
4. **OpenVINS** — `ros2 launch ov_msckf airsim_vio.launch.py
   verbosity:=DEBUG`.

**Critical**: launch OpenVINS **with the drone stationary on the ground**.
Launching mid-flight will cause the filter to dynamic-init at altitude with
wrong scale, then run on IMU dead-reckoning until it diverges to kilometers
of error. See §5.

### 6.2 Verifying init before flight

After launching OpenVINS, before flying, check the log:

```
ros2 node list                              # /ov_msckf/run_subscribe_msckf alive
ros2 topic hz /ov_msckf/poseimu             # publishing
grep "successfully initialized" /tmp/ov_debug.log
grep "ZUPT.*passed disparity" /tmp/ov_debug.log | tail -3
```

You should see:
- `[ZUPT]: passed disparity (0.02 < 1.50, ~330 features)` — drone visually
  stationary, ~330 features tracked.
- `[ZUPT]: accepted |v_IinG| = 0.001` — velocity estimate near zero.
- `[init]: successful initialization in 0.0002 seconds` after the first real
  motion (rotor spin-up or takeoff).

### 6.3 In-flight verification

Watch `MSCKF update (X feats)` lines:

```
grep "MSCKF update" /tmp/ov_debug.log | tail -20
```

- `(0 feats)` repeating → vision is being rejected, filter is dead-reckoning.
  Land, kill OpenVINS, debug.
- `(5–40 feats)` mostly → healthy.

### 6.4 Logging and analysis

Trajectory output: `/mnt/rnd/Shubham/openvins_ws/logs/openvins/ov_estimate.txt`

Format: `timestamp qx qy qz qw px py pz vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z [+ extrinsics]`

Quick divergence check:
```
awk 'NR>1 {print $1, $6, $7, $8}' ov_estimate.txt | tail -20
```
If `px,py,pz` magnitudes blow up (>>100m for a typical mission), the filter
diverged.

### 6.5 T_imu_cam matrix recipes

For a forward-facing camera (Pitch=0 in AirSim), R_CtoI is:

```
[  0,  0,  1 ]
[ -1,  0,  0 ]
[  0, -1,  0 ]
```

For pitch angle θ (negative = nose down) in AirSim NED frame, the rotation
matrix becomes (computed by left-multiplying the forward-facing matrix by
R_y(+|θ|) in FLU body frame, since AirSim NED Y axis is FLU's −Y, so the
sign flips):

```
For θ = -30°:
[  0, -0.500,  0.866 ]
[ -1,  0.000,  0.000 ]
[  0, -0.866, -0.500 ]

For θ = -45°:
[  0, -0.7071,  0.7071 ]
[ -1,  0.0000,  0.0000 ]
[  0, -0.7071, -0.7071 ]

For θ = -60°:
[  0, -0.866,  0.500 ]
[ -1,  0.000,  0.000 ]
[  0, -0.500, -0.866 ]

For θ = -90° (straight down):
[  0, -1.0,  0.0 ]
[ -1,  0.0,  0.0 ]
[  0,  0.0, -1.0 ]
```

Translation `[0.50, 0.00, -0.10]` is unchanged by pitch rotation.

### 6.6 Intrinsics from FOV and resolution

```
fx = fy = (Width / 2) / tan(HFOV_degrees / 2 * π/180)
cx = Width / 2
cy = Height / 2
```

| Resolution | HFOV | fx, fy | cx, cy |
|---|---|---|---|
| 640×480 | 90° | 320.0 | 320.0, 240.0 |
| 640×480 | 62° | 532.5 | 320.0, 240.0 |
| 1280×720 | 62° | 1065.1 | 640.0, 360.0 |
| 1280×960 | 62° | 1065.1 | 640.0, 480.0 |
| 1456×1088 | 62° | 1212.0 | 728.0, 544.0 |

After changing resolution in `settings.json`, **restart AirSim** for the
change to take effect, then update intrinsics in **both** `kalibr_imucam_chain.yaml`
copies.

---

## Quick reference card

| Symptom | First fix |
|---|---|
| Filter never initializes | Wiggle drone before takeoff to satisfy `init_imu_thresh` |
| ZUPT firing in flight | `zupt_only_at_beginning: true` |
| Drift from frame 1 | Check `calib_cam_extrinsics: false` and intrinsics/extrinsics match AirSim |
| Drift starts at altitude X | Loosen chi2 multiplier, increase resolution |
| Drift during pure climb | Tilt camera to −45°, climb on a slope |
| Diverged, won't recover | Land, kill OpenVINS, restart on ground |
| Heavy CPU during tracking | Lower `num_pts`, raise `fast_threshold`, lower `num_opencv_threads` |
| Hover instability | Bump `max_slam`, `max_clones`, ensure `feat_rep_slam: ANCHORED_MSCKF_INVERSE_DEPTH` |

---

## Appendix: parameters NOT in the config but worth knowing

These are OpenVINS defaults you might encounter while reading source code:

- `chi2_multipler` is misspelled in OpenVINS source as well — `multipler` not
  `multiplier`. This is intentional, do not "fix" it.
- The chi-square thresholds themselves come from `boost::math::chi_squared`
  with 95% confidence at the appropriate degrees of freedom (2 for a
  reprojection residual). The `multipler` parameter scales this.
- `init_imu_thresh` is variance-based: it's the standard deviation of the
  accelerometer norm over the init window. 0.5 m/s² means the IMU must see
  noticeable movement, not just hover noise.
