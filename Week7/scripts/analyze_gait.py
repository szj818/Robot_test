#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
步态数据可视化与关节协调分析
========================================

用法：
    python3 analyze_gait.py [csv路径]

默认读取与本脚本同目录下的 gait_data.csv。
输出两个图：
  1. 12 关节角时序曲线（上：左腿，下：右腿），标注步态相位
  2. 关节协调分析（hip_pitch vs knee 联动、roll/pitch 相位差、左右对称性）

步态相位定义（supportPhaseRatio=0 → stepLength=0.5）：
  phase ∈ [0.0, 0.5)  → 左单支撑相（左脚支撑，右脚摆动）
  phase ∈ [0.5, 1.0)  → 右单支撑相（右脚支撑，左脚摆动）
  相位边界附近（±0.02）→ 双支撑过渡（近似标注）
"""

import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# ─── 中文字体支持 ─────────────────────────────────────────────────────────
try:
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
except Exception:
    pass

# ─── 常量 ─────────────────────────────────────────────────────────────────
_SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV   = os.path.join(_SCRIPT_DIR, 'gait_data.csv')
STEP_LENGTH   = 0.5       # 与 ikwalk_engine 保持一致（supportPhaseRatio=0）
DS_HALF_WIDTH = 0.02      # 双支撑边界宽度（相位单位）

LEFT_JOINTS  = ['left_hip_yaw',  'left_hip_roll',  'left_hip_pitch',
                 'left_knee',     'left_ankle_pitch','left_ankle_roll']
RIGHT_JOINTS = ['right_hip_yaw', 'right_hip_roll',  'right_hip_pitch',
                 'right_knee',    'right_ankle_pitch','right_ankle_roll']

JOINT_LABELS = {
    'left_hip_yaw':    'Hip Yaw',    'left_hip_roll':   'Hip Roll',
    'left_hip_pitch':  'Hip Pitch',  'left_knee':        'Knee',
    'left_ankle_pitch':'Ank.Pitch',  'left_ankle_roll':  'Ank.Roll',
    'right_hip_yaw':   'Hip Yaw',    'right_hip_roll':   'Hip Roll',
    'right_hip_pitch': 'Hip Pitch',  'right_knee':       'Knee',
    'right_ankle_pitch':'Ank.Pitch', 'right_ankle_roll': 'Ank.Roll',
}

JOINT_COLORS = ['#E63946', '#457B9D', '#2A9D8F', '#E9C46A', '#F4A261', '#A8DADC']
COLOR_LS = '#D4EDDA'
COLOR_RS = '#D1ECF1'
COLOR_DS = '#FFF3CD'

# ─── 数据加载 ─────────────────────────────────────────────────────────────
def load_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"CSV 文件不存在：{path}\n"
            "请先运行 walk_demo.py 并按 W 键前进录制数据。"
        )
    data = {}
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                data.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in data.items()}

# ─── 相位区域 ─────────────────────────────────────────────────────────────
def _phase_spans(time, phase):
    ls_spans, rs_spans, ds_spans = [], [], []

    def _classify(ph):
        ph_mod = ph % 1.0
        if abs(ph_mod - 0.0) < DS_HALF_WIDTH or abs(ph_mod - 1.0) < DS_HALF_WIDTH:
            return 'DS'
        if abs(ph_mod - STEP_LENGTH) < DS_HALF_WIDTH:
            return 'DS'
        if 0.0 <= ph_mod < STEP_LENGTH:
            return 'LS'
        return 'RS'

    labels = [_classify(p) for p in phase]
    i = 0
    while i < len(time):
        j = i
        cur = labels[i]
        while j < len(time) and labels[j] == cur:
            j += 1
        span = (time[i], time[j - 1])
        if cur == 'LS':
            ls_spans.append(span)
        elif cur == 'RS':
            rs_spans.append(span)
        else:
            ds_spans.append(span)
        i = j
    return ls_spans, rs_spans, ds_spans

def _shade_phases(ax, ls_spans, rs_spans, ds_spans, alpha=0.25):
    for t0, t1 in ls_spans:
        ax.axvspan(t0, t1, color=COLOR_LS, alpha=alpha, lw=0)
    for t0, t1 in rs_spans:
        ax.axvspan(t0, t1, color=COLOR_RS, alpha=alpha, lw=0)
    for t0, t1 in ds_spans:
        ax.axvspan(t0, t1, color=COLOR_DS, alpha=alpha, lw=0)

# ─── 图1：12 关节时序曲线 ────────────────────────────────────────────────
def plot_joint_timeseries(data):
    time  = data['time']
    phase = data['phase']
    ls_spans, rs_spans, ds_spans = _phase_spans(time, phase)

    fig = plt.figure(figsize=(16, 9), facecolor='#1a1a2e')
    fig.suptitle('Humanoid Gait — 12 Joint Angles with Phase Annotation',
                 color='white', fontsize=15, fontweight='bold', y=0.98)

    gs = GridSpec(2, 1, figure=fig, hspace=0.45,
                  left=0.07, right=0.97, top=0.93, bottom=0.09)
    axes = [fig.add_subplot(gs[0]), fig.add_subplot(gs[1])]
    titles = ['Left Leg  (6 DOF)', 'Right Leg  (6 DOF)']
    joint_groups = [LEFT_JOINTS, RIGHT_JOINTS]

    for ax, title, joints in zip(axes, titles, joint_groups):
        ax.set_facecolor('#0d0d1a')
        _shade_phases(ax, ls_spans, rs_spans, ds_spans)
        for jname, color in zip(joints, JOINT_COLORS):
            ax.plot(time, np.degrees(data[jname]),
                    color=color, lw=1.6, label=JOINT_LABELS[jname])
        ax.set_title(title, color='#a0c4ff', fontsize=12, pad=4)
        ax.set_xlabel('Time (s)', color='#cccccc', fontsize=10)
        ax.set_ylabel('Angle (deg)', color='#cccccc', fontsize=10)
        ax.tick_params(colors='#aaaaaa', labelsize=9)
        for sp in ax.spines.values():
            sp.set_edgecolor('#444444')
        ax.grid(True, color='#333355', lw=0.5, linestyle='--')
        leg = ax.legend(loc='upper right', ncol=3, fontsize=8,
                  facecolor='#1a1a2e', edgecolor='#444466',
                  framealpha=0.8)
        for txt in leg.get_texts():
            txt.set_color('white')

    patch_ls = mpatches.Patch(color=COLOR_LS, alpha=0.6,
                               label='Left Single Support  (LS): phase∈[0, 0.5)')
    patch_rs = mpatches.Patch(color=COLOR_RS, alpha=0.6,
                               label='Right Single Support (RS): phase∈[0.5, 1.0)')
    patch_ds = mpatches.Patch(color=COLOR_DS, alpha=0.8,
                               label='Double Support (DS): phase≈0 or 0.5')
    fig_leg = fig.legend(handles=[patch_ls, patch_rs, patch_ds],
               loc='lower center', ncol=3, fontsize=9,
               facecolor='#1a1a2e', edgecolor='#555577',
               framealpha=0.9,
               bbox_to_anchor=(0.5, 0.01))
    for txt in fig_leg.get_texts():
        txt.set_color('white')

    out = os.path.join(_SCRIPT_DIR, 'gait_timeseries.png')
    plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f'[图1] 已保存 {out}')
    return fig

# ─── 互相关估算滞后 ──────────────────────────────────────────────────────
def _phase_lag(sig1, sig2, dt):
    corr = np.correlate(sig1 - sig1.mean(), sig2 - sig2.mean(), mode='full')
    lag  = (np.argmax(corr) - (len(sig1) - 1)) * dt
    return lag

# ─── 图2：协调分析 ────────────────────────────────────────────────────────
def plot_coordination(data):
    time = data['time']
    dt   = float(np.mean(np.diff(time)))

    lhp = np.degrees(data['left_hip_pitch'])
    lk  = np.degrees(data['left_knee'])
    lhr = np.degrees(data['left_hip_roll'])
    rhp = np.degrees(data['right_hip_pitch'])
    rk  = np.degrees(data['right_knee'])

    lag_hpk  = _phase_lag(lhp, lk, dt)
    lag_rollp = _phase_lag(lhr, lhp, dt)

    fig = plt.figure(figsize=(16, 10), facecolor='#1a1a2e')
    fig.suptitle('Gait Coordination Analysis',
                 color='white', fontsize=15, fontweight='bold', y=0.98)
    gs = GridSpec(2, 3, figure=fig, hspace=0.52, wspace=0.38,
                  left=0.07, right=0.97, top=0.92, bottom=0.08)

    # A: hip_pitch vs knee 时序
    axA = fig.add_subplot(gs[0, 0])
    axA.set_facecolor('#0d0d1a')
    axA.plot(time, lhp, color='#E63946', lw=1.5, label='L Hip Pitch')
    axA.plot(time, lk,  color='#2A9D8F', lw=1.5, label='L Knee', linestyle='--')
    axA.set_title(f'Hip Pitch vs Knee  (Left)\nlag = {lag_hpk*1000:.1f} ms',
                  color='#a0c4ff', fontsize=10)
    axA.set_xlabel('Time (s)', color='#cccccc', fontsize=9)
    axA.set_ylabel('Angle (deg)', color='#cccccc', fontsize=9)
    axA.legend(fontsize=8, facecolor='#1a1a2e', edgecolor='#444444')
    axA.grid(True, color='#333355', lw=0.5, linestyle='--')
    axA.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axA.spines.values():
        sp.set_edgecolor('#444444')

    # B: hip_pitch vs knee 相图
    axB = fig.add_subplot(gs[1, 0])
    axB.set_facecolor('#0d0d1a')
    sc = axB.scatter(lhp, lk, c=time, cmap='plasma', s=3, alpha=0.7)
    cbar = fig.colorbar(sc, ax=axB, pad=0.02)
    cbar.ax.yaxis.set_tick_params(color='#aaaaaa', labelsize=7)
    cbar.set_label('Time (s)', color='#aaaaaa', fontsize=8)
    axB.set_title('Hip Pitch – Knee Phase Portrait\n(Left, closed loop = periodic)',
                  color='#a0c4ff', fontsize=10)
    axB.set_xlabel('Hip Pitch (deg)', color='#cccccc', fontsize=9)
    axB.set_ylabel('Knee (deg)', color='#cccccc', fontsize=9)
    axB.grid(True, color='#333355', lw=0.5, linestyle='--')
    axB.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axB.spines.values():
        sp.set_edgecolor('#444444')

    # C: hip_roll vs hip_pitch
    axC = fig.add_subplot(gs[0, 1])
    axC.set_facecolor('#0d0d1a')
    axC.plot(time, lhr, color='#457B9D', lw=1.5, label='L Hip Roll')
    axC.plot(time, lhp, color='#E63946', lw=1.5, label='L Hip Pitch', linestyle='-.')
    axC.set_title(f'Hip Roll vs Pitch  (Left)\nroll lag = {lag_rollp*1000:.1f} ms',
                  color='#a0c4ff', fontsize=10)
    axC.set_xlabel('Time (s)', color='#cccccc', fontsize=9)
    axC.set_ylabel('Angle (deg)', color='#cccccc', fontsize=9)
    axC.legend(fontsize=8, facecolor='#1a1a2e', edgecolor='#444444')
    axC.grid(True, color='#333355', lw=0.5, linestyle='--')
    axC.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axC.spines.values():
        sp.set_edgecolor('#444444')

    # D: Left vs Right hip_pitch
    axD = fig.add_subplot(gs[1, 1])
    axD.set_facecolor('#0d0d1a')
    axD.plot(time, lhp, color='#E63946', lw=1.5, label='L Hip Pitch')
    axD.plot(time, rhp, color='#F4A261', lw=1.5, label='R Hip Pitch', linestyle='--')
    axD.set_title('L vs R Hip Pitch\n(ideal: same shape, 180° offset)',
                  color='#a0c4ff', fontsize=10)
    axD.set_xlabel('Time (s)', color='#cccccc', fontsize=9)
    axD.set_ylabel('Angle (deg)', color='#cccccc', fontsize=9)
    axD.legend(fontsize=8, facecolor='#1a1a2e', edgecolor='#444444')
    axD.grid(True, color='#333355', lw=0.5, linestyle='--')
    axD.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axD.spines.values():
        sp.set_edgecolor('#444444')

    # E: 各关节幅值对比条形图
    axE = fig.add_subplot(gs[0, 2])
    axE.set_facecolor('#0d0d1a')
    joint_pairs = [
        ('left_hip_yaw',  'right_hip_yaw',  'Yaw'),
        ('left_hip_roll', 'right_hip_roll',  'Roll'),
        ('left_hip_pitch','right_hip_pitch', 'H.Pitch'),
        ('left_knee',     'right_knee',      'Knee'),
        ('left_ankle_pitch','right_ankle_pitch','A.Pitch'),
        ('left_ankle_roll', 'right_ankle_roll', 'A.Roll'),
    ]
    x = np.arange(len(joint_pairs))
    w = 0.35
    l_amps = [np.degrees(np.ptp(data[lj])) for lj, _, _ in joint_pairs]
    r_amps = [np.degrees(np.ptp(data[rj])) for _, rj, _ in joint_pairs]
    axE.bar(x - w/2, l_amps, w, label='Left',  color='#E63946', alpha=0.85)
    axE.bar(x + w/2, r_amps, w, label='Right', color='#457B9D', alpha=0.85)
    axE.set_xticks(x)
    axE.set_xticklabels([lbl for _, _, lbl in joint_pairs],
                        rotation=30, ha='right', color='#cccccc', fontsize=8)
    axE.set_title('Joint Amplitude: Left vs Right\n(symmetry check)',
                  color='#a0c4ff', fontsize=10)
    axE.set_ylabel('Peak-to-Peak (deg)', color='#cccccc', fontsize=9)
    axE.legend(fontsize=8, facecolor='#1a1a2e', edgecolor='#444444')
    axE.grid(True, axis='y', color='#333355', lw=0.5, linestyle='--')
    axE.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axE.spines.values():
        sp.set_edgecolor('#444444')

    # F: Knee 对称误差
    axF = fig.add_subplot(gs[1, 2])
    axF.set_facecolor('#0d0d1a')
    sym_err = lk - rk
    axF.plot(time, sym_err, color='#A8DADC', lw=1.2, label='L_knee − R_knee')
    axF.axhline(0, color='#888888', lw=0.8, linestyle='--')
    axF.fill_between(time, sym_err, 0,
                     where=(np.abs(sym_err) > 0.5),
                     color='#E63946', alpha=0.25, label='|err|>0.5°')
    rms = np.sqrt(np.mean(sym_err**2))
    axF.set_title(f'Knee Symmetry Error  (L − R)\nRMS = {rms:.3f} deg',
                  color='#a0c4ff', fontsize=10)
    axF.set_xlabel('Time (s)', color='#cccccc', fontsize=9)
    axF.set_ylabel('Error (deg)', color='#cccccc', fontsize=9)
    axF.legend(fontsize=8, facecolor='#1a1a2e', edgecolor='#444444')
    axF.grid(True, color='#333355', lw=0.5, linestyle='--')
    axF.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in axF.spines.values():
        sp.set_edgecolor('#444444')

    out = os.path.join(_SCRIPT_DIR, 'gait_coordination.png')
    plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f'[图2] 已保存 {out}')
    return fig, lag_hpk, lag_rollp, rms

# ─── 文字分析报告 ─────────────────────────────────────────────────────────
def print_analysis(data, lag_hpk, lag_rollp, knee_rms):
    time = data['time']
    dt   = float(np.mean(np.diff(time)))
    phase_diff = np.diff(data['phase'])
    phase_diff[phase_diff < 0] += 1.0
    freq_real = float(np.mean(phase_diff) / dt)
    period = 1.0 / freq_real if freq_real > 0 else float('nan')

    print('\n' + '=' * 65)
    print('  步态关节协调分析报告')
    print('=' * 65)
    print(f'  录制时长      : {time[-1]:.2f} s')
    print(f'  帧数          : {len(time)} 帧   采样率 ≈ {1/dt:.1f} Hz')
    print(f'  步态频率估计  : {freq_real:.3f} Hz   周期 T ≈ {period:.3f} s')
    print()

    print('【1】hip_pitch 与 knee 的联动关系')
    print(f'  hip_pitch 幅值 : {np.degrees(np.ptp(data["left_hip_pitch"])):.2f}°')
    print(f'  knee      幅值 : {np.degrees(np.ptp(data["left_knee"])):.2f}°')
    print(f'  knee 相对 hip_pitch 的时间滞后 : {lag_hpk*1000:.1f} ms'
          f'  ({lag_hpk/period*360:.1f}°)')
    print('  解释：')
    print('    • IK 中 knee = π − acos(q_knee)，hip_pitch = alpha + A_omega_B，')
    print('      两者均由同一足端目标位置（stepSpline + riseSpline）决定。')
    print('    • 膝关节弯曲主要补偿足端抬高（riseSpline），hip_pitch 控制前向步伐，')
    print('      两者形状互补（knee 大→hip_pitch 大），相位差接近 0 或 180°。')
    print()

    print('【2】hip_roll 与 hip_pitch 的相位关系')
    print(f'  hip_roll  幅值 : {np.degrees(np.ptp(data["left_hip_roll"])):.2f}°')
    print(f'  hip_pitch 幅值 : {np.degrees(np.ptp(data["left_hip_pitch"])):.2f}°')
    print(f'  hip_roll 相对 hip_pitch 的滞后 : {lag_rollp*1000:.1f} ms'
          f'  ({lag_rollp/period*360:.1f}°)')
    print('  解释：')
    print('    • hip_pitch 由 stepSpline 驱动（频率 = 步态频率 f）。')
    print('    • hip_roll  由 swingSpline 驱动（同频率，但 swingPhase=0.25）。')
    print('    • 侧摆比步伐超前约 90°（T/4），确保单支撑相时躯干已移向支撑腿，')
    print('      这是 ZMP 稳定性的经典设计。')
    print()

    print('【3】左右同名关节的对称关系')
    sym_pairs = [
        ('hip_pitch', data['left_hip_pitch'], data['right_hip_pitch']),
        ('knee',      data['left_knee'],      data['right_knee']),
        ('hip_roll',  data['left_hip_roll'],  data['right_hip_roll']),
    ]
    for name, l, r in sym_pairs:
        lag = _phase_lag(np.degrees(l), np.degrees(r), dt)
        print(f'  {name:12s}: R 相对 L 滞后 {lag*1000:+.1f} ms'
              f'  ({lag/period*360:+.1f}°)  理想 ≈ ±{period/2*1000:.0f} ms')
    print(f'  knee 左右对称误差 RMS = {knee_rms:.3f}°')
    print()
    print('  规律：左右腿同名关节波形完全相同，相位差恒为 180°（T/2），')
    print('  体现了双足交替行走的严格对称性（由 phaseRight = phaseLeft + 0.5 保证）。')
    print('=' * 65)

# ─── 主函数 ───────────────────────────────────────────────────────────────
def _phase_lag(sig1, sig2, dt):
    corr = np.correlate(sig1 - sig1.mean(), sig2 - sig2.mean(), mode='full')
    lag  = (np.argmax(corr) - (len(sig1) - 1)) * dt
    return lag

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    print(f'[分析] 读取 CSV：{csv_path}')
    data = load_csv(csv_path)
    print(f'[分析] 共 {len(data["time"])} 帧，时长 {data["time"][-1]:.2f} s')

    fig1 = plot_joint_timeseries(data)
    fig2, lag_hp_k, lag_rp, krms = plot_coordination(data)
    print_analysis(data, lag_hp_k, lag_rp, krms)
    plt.show()

if __name__ == '__main__':
    main()
