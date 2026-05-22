#!/usr/bin/env python3.12
"""
VIO Evaluation Report Generator
================================
Master script that:
  1. Converts ov_estimate.txt to TUM format
  2. Generates trajectory + velocity comparison plots
  3. Computes all evaluation metrics
  4. Produces a PDF report with plots and tables

Usage:
  python3.12 generate_vio_report.py [--logdir /path/to/logs] [--output report.pdf]

Expects these files in logdir:
  - ov_estimate.txt          (OpenVINS state estimate)
  - ov_groundtruth.txt       (GT in TUM format, from ROS2Visualizer)
  - groundtruth_asl.csv      (GT with velocity, from bridge)
"""

import argparse
import os
import sys
import numpy as np
from datetime import datetime

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description='VIO Evaluation Report Generator')
parser.add_argument('--logdir', default='/home/shubham/openvins_ws/logs/openvins',
                    help='Directory containing log files')
parser.add_argument('--output', default=None,
                    help='Output PDF path (default: <logdir>/vio_report_<timestamp>.pdf)')
args = parser.parse_args()

LOGDIR = args.logdir
if args.output:
    OUTPUT_PDF = args.output
else:
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    OUTPUT_PDF = os.path.join(LOGDIR, f'vio_report_{ts}.pdf')

# ---------------------------------------------------------------------------
# Validate input files
# ---------------------------------------------------------------------------
REQUIRED = ['ov_estimate.txt', 'ov_groundtruth.txt', 'groundtruth_asl.csv']
for f in REQUIRED:
    p = os.path.join(LOGDIR, f)
    if not os.path.isfile(p):
        print(f"ERROR: Missing required file: {p}")
        sys.exit(1)

print(f"Log directory : {LOGDIR}")
print(f"Output PDF    : {OUTPUT_PDF}")

# ---------------------------------------------------------------------------
# 1. Load data
# ---------------------------------------------------------------------------
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_tum(path):
    """Load TUM format: ts tx ty tz qx qy qz qw"""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                rows.append([float(x) for x in parts[:8]])
    return np.array(rows)

def load_ov_estimate(path):
    """Load ov_estimate.txt: ts qx qy qz qw px py pz vx vy vz ..."""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            parts = line.strip().split()
            if len(parts) >= 11:
                rows.append([float(x) for x in parts[:11]])
    return np.array(rows)

def load_gt_csv(path):
    """Load groundtruth_asl.csv: ts_ns,qw,qx,qy,qz,px,py,pz,vx,vy,vz,..."""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            parts = line.strip().split(',')
            if len(parts) >= 11:
                rows.append([float(x) for x in parts[:11]])
    return np.array(rows)

print("Loading data...")
gt_tum = load_tum(os.path.join(LOGDIR, 'ov_groundtruth.txt'))
est_raw = load_ov_estimate(os.path.join(LOGDIR, 'ov_estimate.txt'))
gt_csv = load_gt_csv(os.path.join(LOGDIR, 'groundtruth_asl.csv'))

# Convert estimate to TUM: ts px py pz qx qy qz qw
est_tum = np.column_stack([
    est_raw[:, 0],          # ts
    est_raw[:, 5:8],        # px py pz
    est_raw[:, 1:5],        # qx qy qz qw
])
# Extract estimate velocity (global frame)
est_vel = est_raw[:, 8:11]  # vx vy vz

# GT velocity from CSV
gt_csv_t = gt_csv[:, 0] / 1e9   # ns → s
gt_csv_vel = gt_csv[:, 8:11]    # vx vy vz (Z-up + yaw-aligned)

# ---------------------------------------------------------------------------
# 2. Time alignment & interpolation
# ---------------------------------------------------------------------------
print("Aligning trajectories...")

t0 = max(gt_tum[0, 0], est_tum[0, 0])
t1 = min(gt_tum[-1, 0], est_tum[-1, 0])

gt_mask = (gt_tum[:, 0] >= t0) & (gt_tum[:, 0] <= t1)
est_mask = (est_tum[:, 0] >= t0) & (est_tum[:, 0] <= t1)
gt_f = gt_tum[gt_mask]
est_f = est_tum[est_mask]
est_vel_f = est_vel[est_mask[: len(est_vel)]] if len(est_mask) == len(est_vel) else est_vel[(est_raw[:, 0] >= t0) & (est_raw[:, 0] <= t1)]

# Interpolate GT position at estimate timestamps
gt_interp = {}
for i, name in enumerate(['px', 'py', 'pz']):
    gt_interp[name] = interp1d(gt_f[:, 0], gt_f[:, i + 1], kind='linear',
                                fill_value='extrapolate')(est_f[:, 0])

gt_at_est = np.column_stack([gt_interp['px'], gt_interp['py'], gt_interp['pz']])
est_pos = est_f[:, 1:4]

# ---------------------------------------------------------------------------
# 3. Posyaw alignment (position + yaw)
# ---------------------------------------------------------------------------
gt_c = gt_at_est - gt_at_est.mean(axis=0)
est_c = est_pos - est_pos.mean(axis=0)

H = est_c[:, :2].T @ gt_c[:, :2]
U, S, Vt = np.linalg.svd(H)
R2d = Vt.T @ U.T
if np.linalg.det(R2d) < 0:
    Vt[-1, :] *= -1
    R2d = Vt.T @ U.T

yaw = np.arctan2(R2d[1, 0], R2d[0, 0])
c_y, s_y = np.cos(yaw), np.sin(yaw)
R_yaw = np.array([[c_y, -s_y, 0], [s_y, c_y, 0], [0, 0, 1]])

est_aligned = (R_yaw @ est_c.T).T + gt_at_est.mean(axis=0)
pos_errors = np.linalg.norm(est_aligned - gt_at_est, axis=1)

t_rel = est_f[:, 0] - est_f[0, 0]

# ---------------------------------------------------------------------------
# 4. Velocity alignment
# ---------------------------------------------------------------------------
# Interpolate GT velocity at estimate timestamps
gt_vel_mask = (gt_csv_t >= t0) & (gt_csv_t <= t1)
gt_vel_interp = {}
for i, name in enumerate(['vx', 'vy', 'vz']):
    gt_vel_interp[name] = interp1d(gt_csv_t[gt_vel_mask], gt_csv_vel[gt_vel_mask, i],
                                    kind='linear', fill_value='extrapolate')(est_f[:, 0])

gt_vel_at_est = np.column_stack([gt_vel_interp['vx'], gt_vel_interp['vy'], gt_vel_interp['vz']])

# Align estimate velocity with same yaw rotation
est_vel_aligned = (R_yaw @ est_vel_f.T).T

gt_speed = np.linalg.norm(gt_vel_at_est, axis=1)
est_speed = np.linalg.norm(est_vel_f, axis=1)
speed_err = est_speed - gt_speed
vel_err_3d = np.linalg.norm(est_vel_aligned - gt_vel_at_est, axis=1)

# ---------------------------------------------------------------------------
# 5. Compute all metrics
# ---------------------------------------------------------------------------
print("Computing metrics...")

traj_len_gt = np.sum(np.linalg.norm(np.diff(gt_f[:, 1:4], axis=0), axis=1))
traj_len_est = np.sum(np.linalg.norm(np.diff(est_pos, axis=0), axis=1))
duration = t_rel[-1]

metrics = {
    'Trajectory': {
        'GT Length (m)': f'{traj_len_gt:.1f}',
        'VIO Length (m)': f'{traj_len_est:.1f}',
        'Duration (s)': f'{duration:.1f}',
        'Yaw Alignment (deg)': f'{np.degrees(yaw):.1f}',
        'GT Samples': str(len(gt_f)),
        'VIO Samples': str(len(est_f)),
    },
    'Absolute Trajectory Error (m)': {
        'RMSE': f'{np.sqrt(np.mean(pos_errors**2)):.3f}',
        'Mean': f'{np.mean(pos_errors):.3f}',
        'Median': f'{np.median(pos_errors):.3f}',
        'Std': f'{np.std(pos_errors):.3f}',
        'Min': f'{np.min(pos_errors):.3f}',
        'Max': f'{np.max(pos_errors):.3f}',
        'Drift %': f'{np.sqrt(np.mean(pos_errors**2)) / traj_len_gt * 100:.3f}',
    },
    'Velocity Error (m/s)': {
        'Speed RMSE': f'{np.sqrt(np.mean(speed_err**2)):.3f}',
        'Speed Mean Abs': f'{np.mean(np.abs(speed_err)):.3f}',
        '3D Vel RMSE': f'{np.sqrt(np.mean(vel_err_3d**2)):.3f}',
        '3D Vel Mean': f'{np.mean(vel_err_3d):.3f}',
        'Vx RMSE': f'{np.sqrt(np.mean((est_vel_aligned[:,0]-gt_vel_at_est[:,0])**2)):.3f}',
        'Vy RMSE': f'{np.sqrt(np.mean((est_vel_aligned[:,1]-gt_vel_at_est[:,1])**2)):.3f}',
        'Vz RMSE': f'{np.sqrt(np.mean((est_vel_aligned[:,2]-gt_vel_at_est[:,2])**2)):.3f}',
        'Max GT Speed': f'{np.max(gt_speed):.2f}',
    },
}

# RPE computation (simple: segment-based)
rpe_segments = [8, 16, 24, 32, 40]  # meters
rpe_results = {}
cum_dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(est_aligned, axis=0), axis=1))])
for seg_len in rpe_segments:
    rpe_errs = []
    for i in range(len(est_aligned)):
        # Find index where cumulative distance from i exceeds seg_len
        target_dist = cum_dist[i] + seg_len
        j_candidates = np.where(cum_dist[i:] >= target_dist)[0]
        if len(j_candidates) == 0:
            continue
        j = i + j_candidates[0]
        if j >= len(est_aligned):
            continue
        # Relative position error
        est_delta = est_aligned[j] - est_aligned[i]
        gt_delta = gt_at_est[j] - gt_at_est[i]
        rpe_errs.append(np.linalg.norm(est_delta - gt_delta))
    if rpe_errs:
        rpe_results[f'{seg_len}m'] = {
            'median': np.median(rpe_errs),
            'mean': np.mean(rpe_errs),
            'samples': len(rpe_errs),
        }

# ---------------------------------------------------------------------------
# 6. Generate plots (saved to temp files for PDF embedding)
# ---------------------------------------------------------------------------
print("Generating plots...")
plot_paths = {}

# --- XY Trajectory ---
fig, ax = plt.subplots(1, 1, figsize=(8, 6.5))
ax.plot(gt_f[:, 1], gt_f[:, 2], 'b-', linewidth=1.5, label='Ground Truth', alpha=0.7)
ax.plot(est_aligned[:, 0], est_aligned[:, 1], 'r-', linewidth=1.5, label='VIO Estimate', alpha=0.7)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('XY Trajectory (posyaw aligned)')
ax.legend()
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_xy.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['xy'] = p

# --- XYZ vs Time ---
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for i, (ax, lbl) in enumerate(zip(axes, ['X (m)', 'Y (m)', 'Z (m)'])):
    gt_t_rel = gt_f[:, 0] - est_f[0, 0]
    ax.plot(gt_t_rel, gt_f[:, i + 1], 'b-', linewidth=1, label='GT', alpha=0.7)
    ax.plot(t_rel, est_aligned[:, i], 'r-', linewidth=1, label='VIO', alpha=0.7)
    ax.set_ylabel(lbl)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
axes[0].set_title('Position vs Time')
axes[2].set_xlabel('Time (s)')
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_xyz_time.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['xyz_time'] = p

# --- ATE over time ---
fig, ax = plt.subplots(1, 1, figsize=(10, 3.5))
ax.plot(t_rel, pos_errors, 'r-', linewidth=1)
ax.fill_between(t_rel, 0, pos_errors, alpha=0.3, color='red')
ax.axhline(y=np.mean(pos_errors), color='k', linestyle='--', linewidth=1,
           label=f'Mean = {np.mean(pos_errors):.2f} m')
ax.axhline(y=np.sqrt(np.mean(pos_errors**2)), color='orange', linestyle='--', linewidth=1,
           label=f'RMSE = {np.sqrt(np.mean(pos_errors**2)):.2f} m')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position Error (m)')
ax.set_title('Absolute Trajectory Error (ATE)')
ax.legend()
ax.grid(True, alpha=0.3)
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_ate.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['ate'] = p

# --- ATE Histogram ---
fig, ax = plt.subplots(1, 1, figsize=(6, 4))
ax.hist(pos_errors, bins=40, color='steelblue', edgecolor='white', alpha=0.8)
ax.axvline(x=np.mean(pos_errors), color='red', linestyle='--', linewidth=2,
           label=f'Mean = {np.mean(pos_errors):.2f} m')
ax.axvline(x=np.median(pos_errors), color='orange', linestyle='--', linewidth=2,
           label=f'Median = {np.median(pos_errors):.2f} m')
ax.set_xlabel('Position Error (m)')
ax.set_ylabel('Count')
ax.set_title('ATE Distribution')
ax.legend()
ax.grid(True, alpha=0.3)
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_ate_hist.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['ate_hist'] = p

# --- Speed comparison ---
fig, axes = plt.subplots(2, 1, figsize=(10, 5), gridspec_kw={'height_ratios': [3, 1]})
ax = axes[0]
ax.plot(t_rel, gt_speed, 'b-', linewidth=1.2, label='GT Speed', alpha=0.8)
ax.plot(t_rel, est_speed, 'r-', linewidth=1.2, label='VIO Speed', alpha=0.8)
ax.set_ylabel('Speed (m/s)')
ax.set_title('Speed Comparison')
ax.legend()
ax.grid(True, alpha=0.3)
ax2 = axes[1]
ax2.fill_between(t_rel, speed_err, 0, alpha=0.4, color='red')
ax2.plot(t_rel, speed_err, 'r-', linewidth=0.8)
ax2.axhline(0, color='k', linewidth=0.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (m/s)')
ax2.grid(True, alpha=0.3)
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_speed.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['speed'] = p

# --- Per-axis velocity ---
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for i, (ax, lbl) in enumerate(zip(axes, ['Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)'])):
    ax.plot(t_rel, gt_vel_at_est[:, i], 'b-', linewidth=1, label='GT', alpha=0.7)
    ax.plot(t_rel, est_vel_aligned[:, i], 'r-', linewidth=1, label='VIO', alpha=0.7)
    rmse_i = np.sqrt(np.mean((est_vel_aligned[:, i] - gt_vel_at_est[:, i])**2))
    ax.set_ylabel(lbl)
    ax.legend(loc='upper right', fontsize=9, title=f'RMSE={rmse_i:.3f}')
    ax.grid(True, alpha=0.3)
axes[0].set_title('Per-axis Velocity (yaw-aligned)')
axes[2].set_xlabel('Time (s)')
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_vel_xyz.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['vel_xyz'] = p

# --- Velocity error ---
fig, ax = plt.subplots(1, 1, figsize=(10, 3.5))
ax.plot(t_rel, vel_err_3d, 'r-', linewidth=1)
ax.fill_between(t_rel, 0, vel_err_3d, alpha=0.3, color='red')
ax.axhline(y=np.mean(vel_err_3d), color='k', linestyle='--',
           label=f'Mean = {np.mean(vel_err_3d):.3f} m/s')
ax.axhline(y=np.sqrt(np.mean(vel_err_3d**2)), color='orange', linestyle='--',
           label=f'RMSE = {np.sqrt(np.mean(vel_err_3d**2)):.3f} m/s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('3D Velocity Error (m/s)')
ax.set_title('Velocity Error Magnitude')
ax.legend()
ax.grid(True, alpha=0.3)
fig.tight_layout()
p = os.path.join(LOGDIR, '_rpt_vel_err.png')
fig.savefig(p, dpi=150)
plt.close()
plot_paths['vel_err'] = p

# ---------------------------------------------------------------------------
# 7. Generate PDF report
# ---------------------------------------------------------------------------
print("Generating PDF...")

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, cm
from reportlab.lib.colors import HexColor, black, gray, white
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image,
                                 Table, TableStyle, PageBreak, KeepTogether)
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT

W, H_page = A4  # 595 x 842 points

doc = SimpleDocTemplate(
    OUTPUT_PDF,
    pagesize=A4,
    leftMargin=18 * mm,
    rightMargin=18 * mm,
    topMargin=15 * mm,
    bottomMargin=15 * mm,
)

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='Title2', parent=styles['Title'], fontSize=20,
                           spaceAfter=4, textColor=HexColor('#1a1a2e')))
styles.add(ParagraphStyle(name='SectionHead', parent=styles['Heading2'], fontSize=13,
                           spaceBefore=6, spaceAfter=4,
                           textColor=HexColor('#16213e'),
                           borderWidth=1, borderColor=HexColor('#0f3460'),
                           borderPadding=3))
styles.add(ParagraphStyle(name='SubHead', parent=styles['Heading3'], fontSize=11,
                           spaceBefore=8, spaceAfter=4,
                           textColor=HexColor('#1a1a2e')))
styles.add(ParagraphStyle(name='SmallBody', parent=styles['Normal'], fontSize=9,
                           textColor=HexColor('#333333')))
styles.add(ParagraphStyle(name='CenterSmall', parent=styles['Normal'], fontSize=8,
                           alignment=TA_CENTER, textColor=gray))

story = []

# --- Title ---
story.append(Paragraph('VIO Evaluation Report', styles['Title2']))
story.append(Paragraph(
    f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}&nbsp;&nbsp;|&nbsp;&nbsp;'
    f'Log: <font size=8>{LOGDIR}</font>',
    styles['CenterSmall']))
story.append(Spacer(1, 4 * mm))

COMPACT_FONT = 8
COMPACT_PAD = 2

def make_table(title, data_dict, col_widths=None):
    """Create a compact styled table from a dict."""
    s_body = ParagraphStyle('tbl', parent=styles['Normal'], fontSize=COMPACT_FONT,
                             textColor=HexColor('#333333'), leading=COMPACT_FONT + 2)
    s_header = ParagraphStyle('tbl_hdr', parent=s_body, textColor=white)
    table_data = [[Paragraph(f'<b>{title}</b>', s_header), '']]
    for k, v in data_dict.items():
        table_data.append([
            Paragraph(k, s_body),
            Paragraph(f'<b>{v}</b>', s_body),
        ])
    if col_widths is None:
        col_widths = [65 * mm, 40 * mm]
    t = Table(table_data, colWidths=col_widths)
    t.setStyle(TableStyle([
        ('SPAN', (0, 0), (1, 0)),
        ('BACKGROUND', (0, 0), (1, 0), HexColor('#0f3460')),
        ('TEXTCOLOR', (0, 0), (1, 0), white),
        ('FONTSIZE', (0, 0), (-1, -1), COMPACT_FONT),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [HexColor('#f0f0f0'), white]),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#cccccc')),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('LEFTPADDING', (0, 0), (-1, -1), 4),
        ('RIGHTPADDING', (0, 0), (-1, -1), 4),
        ('TOPPADDING', (0, 0), (-1, -1), COMPACT_PAD),
        ('BOTTOMPADDING', (0, 0), (-1, -1), COMPACT_PAD),
    ]))
    return t


def add_image(path, width_mm=160):
    """Add an image scaled to given width."""
    from PIL import Image as PILImage
    im = PILImage.open(path)
    w_px, h_px = im.size
    w = width_mm * mm
    h = w * h_px / w_px
    return Image(path, width=w, height=h)


# Half-page column width for side-by-side tables
HALF_W = 87 * mm
TBL_COL = [58 * mm, 28 * mm]

# --- Section 1: Summary (all tables on page 1) ---
story.append(Paragraph('1. Summary', styles['SectionHead']))

# Row 1: Trajectory Info | Position Error (ATE) — side by side
tbl_traj = make_table('Trajectory Info', metrics['Trajectory'], TBL_COL)
tbl_ate = make_table('Position Error (ATE)', metrics['Absolute Trajectory Error (m)'], TBL_COL)
row1 = Table([[tbl_traj, tbl_ate]], colWidths=[HALF_W, HALF_W])
row1.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP')]))
story.append(row1)
story.append(Spacer(1, 3 * mm))

# Row 2: Velocity Error | RPE — side by side
tbl_vel = make_table('Velocity Error', metrics['Velocity Error (m/s)'], TBL_COL)

# Build RPE as a compact 2-col table
rpe_compact = {}
if rpe_results:
    for seg, vals in rpe_results.items():
        rpe_compact[f'{seg} ({vals["samples"]} pts)'] = f'{vals["median"]:.3f}'
tbl_rpe = make_table('Relative Pose Error (median m)', rpe_compact, TBL_COL)

row2 = Table([[tbl_vel, tbl_rpe]], colWidths=[HALF_W, HALF_W])
row2.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP')]))
story.append(row2)
story.append(Spacer(1, 3 * mm))

# Assessment box on page 1
drift_pct = float(metrics['Absolute Trajectory Error (m)']['Drift %'])
ate_rmse = float(metrics['Absolute Trajectory Error (m)']['RMSE'])
speed_rmse = float(metrics['Velocity Error (m/s)']['Speed RMSE'])

def grade(val, thresholds, labels):
    for t, l in zip(thresholds, labels):
        if val <= t:
            return l
    return labels[-1]

pos_grade = grade(drift_pct, [0.5, 1.0, 2.0, 5.0],
                  ['Excellent', 'Good', 'Acceptable', 'Marginal', 'Poor'])
vel_grade = grade(speed_rmse, [0.2, 0.5, 1.0, 2.0],
                  ['Excellent', 'Good', 'Acceptable', 'Marginal', 'Poor'])

grade_colors = {
    'Excellent': HexColor('#27ae60'), 'Good': HexColor('#2ecc71'),
    'Acceptable': HexColor('#f39c12'), 'Marginal': HexColor('#e67e22'),
    'Poor': HexColor('#e74c3c'),
}

s_assess = ParagraphStyle('assess', parent=styles['Normal'], fontSize=COMPACT_FONT,
                           textColor=HexColor('#333333'), leading=COMPACT_FONT + 2)
s_assess_hdr = ParagraphStyle('assess_hdr', parent=s_assess, textColor=white)
assessment_data = [
    [Paragraph('<b>Metric</b>', s_assess_hdr),
     Paragraph('<b>Value</b>', s_assess_hdr),
     Paragraph('<b>Grade</b>', s_assess_hdr)],
    [Paragraph('Position Drift', s_assess),
     Paragraph(f'{drift_pct:.3f}%', s_assess),
     Paragraph(f'<b>{pos_grade}</b>', s_assess)],
    [Paragraph('ATE RMSE', s_assess),
     Paragraph(f'{ate_rmse:.3f} m', s_assess),
     Paragraph('—', s_assess)],
    [Paragraph('Speed RMSE', s_assess),
     Paragraph(f'{speed_rmse:.3f} m/s', s_assess),
     Paragraph(f'<b>{vel_grade}</b>', s_assess)],
]
assess_tbl = Table(assessment_data, colWidths=[45 * mm, 35 * mm, 35 * mm])
assess_style = [
    ('BACKGROUND', (0, 0), (-1, 0), HexColor('#0f3460')),
    ('TEXTCOLOR', (0, 0), (-1, 0), white),
    ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#cccccc')),
    ('ROWBACKGROUNDS', (0, 1), (-1, -1), [HexColor('#f0f0f0'), white]),
    ('FONTSIZE', (0, 0), (-1, -1), COMPACT_FONT),
    ('LEFTPADDING', (0, 0), (-1, -1), 4),
    ('TOPPADDING', (0, 0), (-1, -1), COMPACT_PAD),
    ('BOTTOMPADDING', (0, 0), (-1, -1), COMPACT_PAD),
]
for row_idx, g in [(1, pos_grade), (3, vel_grade)]:
    assess_style.append(('TEXTCOLOR', (2, row_idx), (2, row_idx), grade_colors.get(g, black)))
assess_tbl.setStyle(TableStyle(assess_style))

story.append(Paragraph('Overall Assessment', styles['SubHead']))
story.append(assess_tbl)
story.append(Spacer(1, 2 * mm))
story.append(Paragraph(
    '<i>Grading: Drift &lt;0.5%=Excellent, &lt;1%=Good, &lt;2%=Acceptable. '
    'Speed RMSE &lt;0.2=Excellent, &lt;0.5=Good, &lt;1.0=Acceptable.</i>',
    styles['CenterSmall']))

story.append(PageBreak())

# --- Page 2: Trajectory Plots ---
story.append(Paragraph('2. Trajectory Comparison', styles['SectionHead']))
story.append(add_image(plot_paths['xy'], 130))
story.append(Spacer(1, 2 * mm))
story.append(add_image(plot_paths['xyz_time'], 155))

story.append(PageBreak())

# --- Page 3: ATE + Histogram ---
story.append(Paragraph('3. Absolute Trajectory Error', styles['SectionHead']))
story.append(add_image(plot_paths['ate'], 160))
story.append(Spacer(1, 3 * mm))
story.append(add_image(plot_paths['ate_hist'], 110))

story.append(Spacer(1, 4 * mm))

# --- Page 3 continued: Speed ---
story.append(Paragraph('4. Speed Comparison', styles['SectionHead']))
story.append(add_image(plot_paths['speed'], 160))

story.append(PageBreak())

# --- Page 4: Velocity per-axis + error ---
story.append(Paragraph('5. Per-axis Velocity', styles['SectionHead']))
story.append(add_image(plot_paths['vel_xyz'], 160))
story.append(Spacer(1, 3 * mm))
story.append(Paragraph('6. Velocity Error', styles['SectionHead']))
story.append(add_image(plot_paths['vel_err'], 160))

# --- Section 6: Assessment ---
drift_pct = float(metrics['Absolute Trajectory Error (m)']['Drift %'])
ate_rmse = float(metrics['Absolute Trajectory Error (m)']['RMSE'])
speed_rmse = float(metrics['Velocity Error (m/s)']['Speed RMSE'])

def grade(val, thresholds, labels):
    for t, l in zip(thresholds, labels):
        if val <= t:
            return l
    return labels[-1]

pos_grade = grade(drift_pct, [0.5, 1.0, 2.0, 5.0],
                  ['Excellent', 'Good', 'Acceptable', 'Marginal', 'Poor'])
vel_grade = grade(speed_rmse, [0.2, 0.5, 1.0, 2.0],
                  ['Excellent', 'Good', 'Acceptable', 'Marginal', 'Poor'])

story.append(Paragraph('6. Overall Assessment', styles['SectionHead']))

s_hdr_white = ParagraphStyle('assess_hdr2', parent=styles['SmallBody'], textColor=white)
assessment_data = [
    [Paragraph('<b>Metric</b>', s_hdr_white),
     Paragraph('<b>Value</b>', s_hdr_white),
     Paragraph('<b>Grade</b>', s_hdr_white)],
    [Paragraph('Position Drift', styles['SmallBody']),
     Paragraph(f'{drift_pct:.3f}%', styles['SmallBody']),
     Paragraph(f'<b>{pos_grade}</b>', styles['SmallBody'])],
    [Paragraph('ATE RMSE', styles['SmallBody']),
     Paragraph(f'{ate_rmse:.3f} m', styles['SmallBody']),
     Paragraph('', styles['SmallBody'])],
    [Paragraph('Speed RMSE', styles['SmallBody']),
     Paragraph(f'{speed_rmse:.3f} m/s', styles['SmallBody']),
     Paragraph(f'<b>{vel_grade}</b>', styles['SmallBody'])],
]

grade_colors = {
    'Excellent': HexColor('#27ae60'),
    'Good': HexColor('#2ecc71'),
    'Acceptable': HexColor('#f39c12'),
    'Marginal': HexColor('#e67e22'),
    'Poor': HexColor('#e74c3c'),
}

assess_tbl = Table(assessment_data, colWidths=[60 * mm, 50 * mm, 50 * mm])
assess_style = [
    ('BACKGROUND', (0, 0), (-1, 0), HexColor('#0f3460')),
    ('TEXTCOLOR', (0, 0), (-1, 0), white),
    ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#cccccc')),
    ('ROWBACKGROUNDS', (0, 1), (-1, -1), [HexColor('#f0f0f0'), white]),
    ('FONTSIZE', (0, 0), (-1, -1), 10),
    ('LEFTPADDING', (0, 0), (-1, -1), 8),
    ('TOPPADDING', (0, 0), (-1, -1), 4),
    ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
]
# Color the grade cells
for row_idx, g in [(1, pos_grade), (3, vel_grade)]:
    assess_style.append(('TEXTCOLOR', (2, row_idx), (2, row_idx), grade_colors.get(g, black)))

assess_tbl.setStyle(TableStyle(assess_style))
story.append(assess_tbl)

story.append(Spacer(1, 8 * mm))
story.append(Paragraph(
    '<i>Grading: Position drift &lt;0.5% = Excellent, &lt;1% = Good, &lt;2% = Acceptable. '
    'Speed RMSE &lt;0.2 m/s = Excellent, &lt;0.5 m/s = Good, &lt;1.0 m/s = Acceptable.</i>',
    styles['CenterSmall']))

# Build PDF
doc.build(story)

# Clean up temp plot files
for p in plot_paths.values():
    if os.path.exists(p):
        os.remove(p)

print(f"\nReport generated: {OUTPUT_PDF}")
