import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

outdir = '/home/shubham/openvins_ws/logs/openvins'

# --- Load GT from ASL CSV ---
# Columns: ts_ns, qw, qx, qy, qz, px, py, pz, vx, vy, vz, ...
gt_raw = []
with open(os.path.join(outdir, 'groundtruth_asl.csv')) as f:
    for line in f:
        if line.startswith('#'): continue
        parts = line.strip().split(',')
        if len(parts) >= 11:
            gt_raw.append([float(x) for x in parts[:11]])
gt_raw = np.array(gt_raw)
gt_t = gt_raw[:, 0] / 1e9  # ns to seconds
gt_vx, gt_vy, gt_vz = gt_raw[:, 8], gt_raw[:, 9], gt_raw[:, 10]
gt_speed = np.sqrt(gt_vx**2 + gt_vy**2 + gt_vz**2)

# --- Load OpenVINS estimate ---
# Columns: ts, qx, qy, qz, qw, px, py, pz, vx, vy, vz, ...
est_raw = []
with open(os.path.join(outdir, 'ov_estimate.txt')) as f:
    for line in f:
        if line.startswith('#'): continue
        parts = line.strip().split()
        if len(parts) >= 11:
            est_raw.append([float(x) for x in parts[:11]])
est_raw = np.array(est_raw)
est_t = est_raw[:, 0]
est_vx, est_vy, est_vz = est_raw[:, 9], est_raw[:, 10], est_raw[:, 11] if est_raw.shape[1] > 11 else (est_raw[:,9], est_raw[:,10], np.zeros_like(est_raw[:,9]))

# Wait - let me recheck columns: ts(1) q(4) p(3) v(3) = indices 0, 1-4, 5-7, 8-10
est_vx = est_raw[:, 8]
est_vy = est_raw[:, 9]
est_vz = est_raw[:, 10]
est_speed = np.sqrt(est_vx**2 + est_vy**2 + est_vz**2)

# --- Time overlap ---
t0 = max(gt_t[0], est_t[0])
t1 = min(gt_t[-1], est_t[-1])

gt_mask = (gt_t >= t0) & (gt_t <= t1)
est_mask = (est_t >= t0) & (est_t <= t1)

gt_tf = gt_t[gt_mask]
est_tf = est_t[est_mask]

# Interpolate GT velocity at estimate timestamps
interp_gt_vx = interp1d(gt_tf, gt_vx[gt_mask], fill_value='extrapolate')
interp_gt_vy = interp1d(gt_tf, gt_vy[gt_mask], fill_value='extrapolate')
interp_gt_vz = interp1d(gt_tf, gt_vz[gt_mask], fill_value='extrapolate')
interp_gt_speed = interp1d(gt_tf, gt_speed[gt_mask], fill_value='extrapolate')

gt_vx_at_est = interp_gt_vx(est_tf)
gt_vy_at_est = interp_gt_vy(est_tf)
gt_vz_at_est = interp_gt_vz(est_tf)
gt_speed_at_est = interp_gt_speed(est_tf)

est_vx_f = est_vx[est_mask]
est_vy_f = est_vy[est_mask]
est_vz_f = est_vz[est_mask]
est_speed_f = est_speed[est_mask]

t_rel = est_tf - est_tf[0]

# --- Yaw alignment for per-axis comparison ---
# Use same Procrustes approach on velocity XY
H = np.column_stack([est_vx_f, est_vy_f]).T @ np.column_stack([gt_vx_at_est, gt_vy_at_est])
U, S, Vt = np.linalg.svd(H)
R2d = Vt.T @ U.T
if np.linalg.det(R2d) < 0:
    Vt[-1,:] *= -1
    R2d = Vt.T @ U.T
yaw = np.arctan2(R2d[1,0], R2d[0,0])
c, s = np.cos(yaw), np.sin(yaw)

# Rotate estimate velocity to GT frame
est_vx_aligned = c * est_vx_f - s * est_vy_f
est_vy_aligned = s * est_vx_f + c * est_vy_f
est_vz_aligned = est_vz_f  # Z unchanged

print(f"Yaw alignment: {np.degrees(yaw):.1f} deg")

# ============ PLOT 1: Speed comparison ============
fig, axes = plt.subplots(2, 1, figsize=(12, 7), gridspec_kw={'height_ratios': [3, 1]})
ax = axes[0]
ax.plot(t_rel, gt_speed_at_est, 'b-', linewidth=1.2, label='GT Speed', alpha=0.8)
ax.plot(t_rel, est_speed_f, 'r-', linewidth=1.2, label='VIO Speed', alpha=0.8)
ax.set_ylabel('Speed (m/s)', fontsize=12)
ax.set_title('Speed Comparison (frame-invariant)', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)

ax2 = axes[1]
speed_err = est_speed_f - gt_speed_at_est
ax2.fill_between(t_rel, speed_err, 0, alpha=0.4, color='red')
ax2.plot(t_rel, speed_err, 'r-', linewidth=0.8)
ax2.axhline(0, color='k', linewidth=0.5)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Error (m/s)', fontsize=12)
ax2.set_title(f'Speed Error — RMSE: {np.sqrt(np.mean(speed_err**2)):.3f} m/s, Mean: {np.mean(np.abs(speed_err)):.3f} m/s', fontsize=11)
ax2.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_speed_comparison.png'), dpi=150)
plt.close()

# ============ PLOT 2: Per-axis velocity (yaw-aligned) ============
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
components = [
    ('Vx (m/s)', gt_vx_at_est, est_vx_aligned),
    ('Vy (m/s)', gt_vy_at_est, est_vy_aligned),
    ('Vz (m/s)', gt_vz_at_est, est_vz_aligned),
]
for ax, (label, gt_v, est_v) in zip(axes, components):
    ax.plot(t_rel, gt_v, 'b-', linewidth=1, label='GT', alpha=0.7)
    ax.plot(t_rel, est_v, 'r-', linewidth=1, label='VIO', alpha=0.7)
    rmse = np.sqrt(np.mean((est_v - gt_v)**2))
    ax.set_ylabel(label, fontsize=11)
    ax.legend(fontsize=10, loc='upper right', title=f'RMSE={rmse:.3f} m/s')
    ax.grid(True, alpha=0.3)
axes[0].set_title(f'Per-axis Velocity (yaw-aligned by {np.degrees(yaw):.1f}°)', fontsize=14)
axes[2].set_xlabel('Time (s)', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_velocity_xyz.png'), dpi=150)
plt.close()

# ============ PLOT 3: Velocity error over time ============
fig, ax = plt.subplots(1, 1, figsize=(12, 4))
vel_err_3d = np.sqrt((est_vx_aligned - gt_vx_at_est)**2 + 
                      (est_vy_aligned - gt_vy_at_est)**2 + 
                      (est_vz_aligned - gt_vz_at_est)**2)
ax.plot(t_rel, vel_err_3d, 'r-', linewidth=1)
ax.fill_between(t_rel, 0, vel_err_3d, alpha=0.3, color='red')
rmse_3d = np.sqrt(np.mean(vel_err_3d**2))
ax.axhline(y=np.mean(vel_err_3d), color='k', linestyle='--', label=f'Mean = {np.mean(vel_err_3d):.3f} m/s')
ax.axhline(y=rmse_3d, color='orange', linestyle='--', label=f'RMSE = {rmse_3d:.3f} m/s')
ax.set_xlabel('Time (s)', fontsize=12)
ax.set_ylabel('3D Velocity Error (m/s)', fontsize=12)
ax.set_title('Velocity Error Magnitude over Time', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_velocity_error.png'), dpi=150)
plt.close()

print(f"\nVelocity Summary:")
print(f"  Speed RMSE:     {np.sqrt(np.mean(speed_err**2)):.3f} m/s")
print(f"  3D Vel RMSE:    {rmse_3d:.3f} m/s")
print(f"  Vx RMSE:        {np.sqrt(np.mean((est_vx_aligned-gt_vx_at_est)**2)):.3f} m/s")
print(f"  Vy RMSE:        {np.sqrt(np.mean((est_vy_aligned-gt_vy_at_est)**2)):.3f} m/s")
print(f"  Vz RMSE:        {np.sqrt(np.mean((est_vz_aligned-gt_vz_at_est)**2)):.3f} m/s")
print(f"  Max GT speed:   {np.max(gt_speed_at_est):.2f} m/s")
print(f"  Max VIO speed:  {np.max(est_speed_f):.2f} m/s")
print(f"\nPlots saved to {outdir}/plot_speed_*.png and plot_velocity_*.png")
