import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

outdir = '/home/shubham/openvins_ws/logs/openvins'

# --- Load GT ---
gt = []
with open(os.path.join(outdir, 'ov_groundtruth.txt')) as f:
    for line in f:
        if line.startswith('#'): continue
        parts = line.strip().split()
        if len(parts) >= 8:
            gt.append([float(x) for x in parts])
gt = np.array(gt)

# --- Load Estimate (TUM format) ---
est = []
with open(os.path.join(outdir, 'ov_est_tum.txt')) as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) >= 8:
            est.append([float(x) for x in parts])
est = np.array(est)

# columns: 0=ts, 1=px, 2=py, 3=pz, 4=qx, 5=qy, 6=qz, 7=qw

# --- Time-align: find overlapping time range ---
t0 = max(gt[0,0], est[0,0])
t1 = min(gt[-1,0], est[-1,0])
gt_mask = (gt[:,0] >= t0) & (gt[:,0] <= t1)
est_mask = (est[:,0] >= t0) & (est[:,0] <= t1)
gt_f = gt[gt_mask]
est_f = est[est_mask]

# --- Simple posyaw alignment (find best yaw + translation) ---
# Interpolate GT at estimate timestamps
from scipy.interpolate import interp1d
gt_interp_x = interp1d(gt_f[:,0], gt_f[:,1], kind='linear', fill_value='extrapolate')
gt_interp_y = interp1d(gt_f[:,0], gt_f[:,2], kind='linear', fill_value='extrapolate')
gt_interp_z = interp1d(gt_f[:,0], gt_f[:,3], kind='linear', fill_value='extrapolate')

gt_at_est = np.column_stack([
    gt_interp_x(est_f[:,0]),
    gt_interp_y(est_f[:,0]),
    gt_interp_z(est_f[:,0])
])
est_pos = est_f[:, 1:4]

# Center both
gt_c = gt_at_est - gt_at_est.mean(axis=0)
est_c = est_pos - est_pos.mean(axis=0)

# Find best yaw (rotation about Z)
# Using Procrustes on XY only
H = est_c[:,:2].T @ gt_c[:,:2]
U, S, Vt = np.linalg.svd(H)
R2d = Vt.T @ U.T
if np.linalg.det(R2d) < 0:
    Vt[-1,:] *= -1
    R2d = Vt.T @ U.T

yaw = np.arctan2(R2d[1,0], R2d[0,0])
c, s = np.cos(yaw), np.sin(yaw)
R_yaw = np.array([[c,-s,0],[s,c,0],[0,0,1]])

est_aligned = (R_yaw @ est_c.T).T + gt_at_est.mean(axis=0)
errors = np.linalg.norm(est_aligned - gt_at_est, axis=1)

t_rel = est_f[:,0] - est_f[0,0]  # relative time in seconds

# ============ PLOT 1: XY Trajectory ============
fig, ax = plt.subplots(1, 1, figsize=(10, 8))
ax.plot(gt_f[:,1], gt_f[:,2], 'b-', linewidth=1.5, label='Ground Truth', alpha=0.7)
ax.plot(est_aligned[:,0], est_aligned[:,1], 'r-', linewidth=1.5, label='VIO Estimate (aligned)', alpha=0.7)
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_title('XY Trajectory (posyaw aligned)', fontsize=14)
ax.legend(fontsize=12)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_xy_trajectory.png'), dpi=150)
plt.close()

# ============ PLOT 2: XYZ vs Time ============
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
labels = ['X (m)', 'Y (m)', 'Z (m)']
for i, (ax, lbl) in enumerate(zip(axes, labels)):
    gt_t_rel = gt_f[:,0] - est_f[0,0]
    ax.plot(gt_t_rel, gt_f[:, i+1], 'b-', linewidth=1, label='GT', alpha=0.7)
    ax.plot(t_rel, est_aligned[:, i], 'r-', linewidth=1, label='VIO', alpha=0.7)
    ax.set_ylabel(lbl, fontsize=11)
    ax.legend(fontsize=10, loc='upper right')
    ax.grid(True, alpha=0.3)
axes[0].set_title('Position vs Time (posyaw aligned)', fontsize=14)
axes[2].set_xlabel('Time (s)', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_xyz_vs_time.png'), dpi=150)
plt.close()

# ============ PLOT 3: ATE over time ============
fig, ax = plt.subplots(1, 1, figsize=(12, 4))
ax.plot(t_rel, errors, 'r-', linewidth=1)
ax.fill_between(t_rel, 0, errors, alpha=0.3, color='red')
ax.axhline(y=np.mean(errors), color='k', linestyle='--', linewidth=1, label=f'Mean = {np.mean(errors):.2f} m')
ax.axhline(y=np.sqrt(np.mean(errors**2)), color='orange', linestyle='--', linewidth=1, label=f'RMSE = {np.sqrt(np.mean(errors**2)):.2f} m')
ax.set_xlabel('Time (s)', fontsize=12)
ax.set_ylabel('Position Error (m)', fontsize=12)
ax.set_title('Absolute Trajectory Error (ATE) over Time', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_ate_over_time.png'), dpi=150)
plt.close()

# ============ PLOT 4: Error histogram ============
fig, ax = plt.subplots(1, 1, figsize=(8, 5))
ax.hist(errors, bins=40, color='steelblue', edgecolor='white', alpha=0.8)
ax.axvline(x=np.mean(errors), color='red', linestyle='--', linewidth=2, label=f'Mean = {np.mean(errors):.2f} m')
ax.axvline(x=np.median(errors), color='orange', linestyle='--', linewidth=2, label=f'Median = {np.median(errors):.2f} m')
ax.set_xlabel('Position Error (m)', fontsize=12)
ax.set_ylabel('Count', fontsize=12)
ax.set_title('ATE Distribution', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(outdir, 'plot_ate_histogram.png'), dpi=150)
plt.close()

# Print summary
print("Plots saved:")
print(f"  {outdir}/plot_xy_trajectory.png")
print(f"  {outdir}/plot_xyz_vs_time.png")
print(f"  {outdir}/plot_ate_over_time.png")
print(f"  {outdir}/plot_ate_histogram.png")
print(f"\nSummary:")
print(f"  Trajectory length (GT): {np.sum(np.linalg.norm(np.diff(gt_f[:,1:4], axis=0), axis=1)):.1f} m")
print(f"  Trajectory length (EST): {np.sum(np.linalg.norm(np.diff(est_pos, axis=0), axis=1)):.1f} m")
print(f"  Duration: {t_rel[-1]:.1f} s")
print(f"  ATE RMSE: {np.sqrt(np.mean(errors**2)):.3f} m")
print(f"  ATE Mean: {np.mean(errors):.3f} m")
print(f"  ATE Max:  {np.max(errors):.3f} m")
print(f"  Drift %%:  {np.sqrt(np.mean(errors**2)) / np.sum(np.linalg.norm(np.diff(gt_f[:,1:4], axis=0), axis=1)) * 100:.2f}%")
