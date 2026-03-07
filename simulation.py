#!/usr/bin/env python3
"""
TCM ZVS 三相仿真 - 带图表
"""
import math
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False

# 参数
g_out_freq_hz = 50.0      # 输出频率 50Hz
g_voltage_output = 0.8    # 电压控制 0~1
g_fsw_center = 40000.0    # 中心频率 40kHz
g_fsw_delta = 20000.0     # 频率变化量 ±20kHz

# 三相相位偏移
PHASE_OFFSET_A = 0.0
PHASE_OFFSET_B = 120.0
PHASE_OFFSET_C = 240.0

# 仿真参数
t_step = 0.0001  # 0.1ms步长
t_total = 0.04   # 40ms = 2个50Hz周期

# 数据存储
times = []
fsw_a, fsw_b, fsw_c = [], [], []
arr_a, arr_b, arr_c = [], [], []

# 仿真循环
sample_count = int(t_total / t_step)
g_phase_deg = 0.0

for i in range(sample_count + 1):
    t_ms = i * t_step * 1000
    
    # 计算三相频率
    sin_a = math.sin(math.radians(g_phase_deg + PHASE_OFFSET_A))
    sin_b = math.sin(math.radians(g_phase_deg + PHASE_OFFSET_B))
    sin_c = math.sin(math.radians(g_phase_deg + PHASE_OFFSET_C))
    
    # TCM ZVS频率计算
    f_a = g_fsw_center + g_fsw_delta * sin_a * g_voltage_output
    f_b = g_fsw_center + g_fsw_delta * sin_b * g_voltage_output
    f_c = g_fsw_center + g_fsw_delta * sin_c * g_voltage_output
    
    # ARR值 (80MHz)
    arr_a_val = int(80000000.0 / f_a) - 1
    arr_b_val = int(80000000.0 / f_b) - 1
    arr_c_val = int(80000000.0 / f_c) - 1
    
    times.append(t_ms)
    fsw_a.append(f_a / 1000)
    fsw_b.append(f_b / 1000)
    fsw_c.append(f_c / 1000)
    arr_a.append(arr_a_val)
    arr_b.append(arr_b_val)
    arr_c.append(arr_c_val)
    
    # 更新相位
    t_s = 1.0 / 20000.0
    g_phase_deg += g_out_freq_hz * 360.0 * t_s * (t_step / t_s)
    if g_phase_deg >= 360.0:
        g_phase_deg -= 360.0

# 创建图表
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle('TCM ZVS 三相仿真 (TIM1/TIM15/TIM16 独立频率)', fontsize=14)

# 图1: 三相开关频率
ax1 = axes[0]
ax1.plot(times, fsw_a, 'r-', linewidth=1.5, label='A相 (TIM1) 0°')
ax1.plot(times, fsw_b, 'g-', linewidth=1.5, label='B相 (TIM15) 120°')
ax1.plot(times, fsw_c, 'b-', linewidth=1.5, label='C相 (TIM16) 240°')
ax1.set_ylabel('f_sw (kHz)')
ax1.set_title('三相开关频率 (f_sw = 40kHz + 20kHz × sin × 0.8)')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=40, color='k', linestyle='--', linewidth=1, label='中心频率')
ax1.set_ylim(20, 60)
ax1.legend(loc='upper right')

# 图2: ARR值
ax2 = axes[1]
ax2.plot(times, arr_a, 'r-', linewidth=1.5, label='A相 ARR')
ax2.plot(times, arr_b, 'g-', linewidth=1.5, label='B相 ARR')
ax2.plot(times, arr_c, 'b-', linewidth=1.5, label='C相 ARR')
ax2.set_ylabel('ARR')
ax2.set_xlabel('时间 (ms)')
ax2.set_title('TIM1/TIM15/TIM16 ARR值 (80MHz时钟)')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='upper right')

plt.tight_layout()
plt.savefig('tcm_zvs_3phase_simulation.png', dpi=150, bbox_inches='tight')
plt.show()

# 打印关键信息
print("=" * 60)
print("TCM ZVS 三相仿真完成!")
print("=" * 60)
print(f"A相 (TIM1):   相位 0°   频率 {min(fsw_a):.1f}~{max(fsw_a):.1f} kHz")
print(f"B相 (TIM15):  相位 120° 频率 {min(fsw_b):.1f}~{max(fsw_b):.1f} kHz")
print(f"C相 (TIM16):  相位 240° 频率 {min(fsw_c):.1f}~{max(fsw_c):.1f} kHz")
print("=" * 60)
print(f"中心频率: 40 kHz")
print(f"电压控制: {g_voltage_output*100:.0f}%")
print(f"占空比: 固定 50%")
print("=" * 60)
print("图表已保存到: tcm_zvs_3phase_simulation.png")