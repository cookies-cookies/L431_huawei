"""
TCM ZVS Three-Phase PWM Simulation - Exact copy of STM32 main.c logic
"""

import numpy as np
import matplotlib.pyplot as plt
import math

# Parameters
TIM1_IRQ_FREQ = 200000.0   # TIM1中断频率 200kHz (100MHz/500)
TIM7_IRQ_FREQ = 20000.0    # TIM7中断频率 20kHz (100MHz/5000)
SIMULATION_TIME = 0.020

# TCM ZVS Parameters
g_out_freq_hz = 50.0
g_fsw_center = 5000.0
g_fsw_delta = 3000.0
g_voltage_output = 0.8
g_duty_cycle = 0.1  # 测试低占空比

# Global Variables
g_phase_cnt_a = 0
g_phase_cnt_b = 0
g_phase_cnt_c = 0
g_period_a = 100
g_period_b = 100
g_period_c = 100
g_high_cnt_a = 50
g_high_cnt_b = 50
g_high_cnt_c = 50
g_state_a = 0
g_state_b = 0
g_state_c = 0
g_ah = 0
g_al = 0
g_bh = 0
g_bl = 0
g_ch = 0
g_cl = 0
phase_deg = 0.0

STATE_HIGH = 0
STATE_LOW = 2

def TIM7_IRQHandler():
    global phase_deg, g_period_a, g_period_b, g_period_c
    global g_high_cnt_a, g_high_cnt_b, g_high_cnt_c
    global g_phase_cnt_a, g_phase_cnt_b, g_phase_cnt_c
    global g_ah, g_al, g_bh, g_bl, g_ch, g_cl
    global g_state_a, g_state_b, g_state_c
    
    phase_deg += g_out_freq_hz * 360.0 / 20000.0
    if phase_deg >= 360.0:
        phase_deg -= 360.0
    
    phase_rad = math.radians(phase_deg)
    sin_a = math.sin(phase_rad)
    cos_a = math.cos(phase_rad)
    sin_b = -0.5 * sin_a + 0.866025404 * cos_a
    sin_c = -0.5 * sin_a - 0.866025404 * cos_a
    
    f_a = g_fsw_center + g_fsw_delta * sin_a * g_voltage_output
    f_b = g_fsw_center + g_fsw_delta * sin_b * g_voltage_output
    f_c = g_fsw_center + g_fsw_delta * sin_c * g_voltage_output
    
    new_period_a = int(TIM1_IRQ_FREQ / f_a)
    new_period_b = int(TIM1_IRQ_FREQ / f_b)
    new_period_c = int(TIM1_IRQ_FREQ / f_c)
    
    new_high_a = int(new_period_a * g_duty_cycle)
    new_high_b = int(new_period_b * g_duty_cycle)
    new_high_c = int(new_period_c * g_duty_cycle)
    
    # 修复：同步counter和HIGH状态
    if g_phase_cnt_a >= new_period_a:
        g_phase_cnt_a = 0
        g_al = 0
        g_ah = 1
        g_state_a = STATE_HIGH
    elif g_state_a == STATE_HIGH and g_phase_cnt_a >= new_high_a:
        g_ah = 0
        g_al = 1
        g_state_a = STATE_LOW
    g_period_a = new_period_a
    g_high_cnt_a = new_high_a
    
    if g_phase_cnt_b >= new_period_b:
        g_phase_cnt_b = 0
        g_bl = 0
        g_bh = 1
        g_state_b = STATE_HIGH
    elif g_state_b == STATE_HIGH and g_phase_cnt_b >= new_high_b:
        g_bh = 0
        g_bl = 1
        g_state_b = STATE_LOW
    g_period_b = new_period_b
    g_high_cnt_b = new_high_b
    
    if g_phase_cnt_c >= new_period_c:
        g_phase_cnt_c = 0
        g_cl = 0
        g_ch = 1
        g_state_c = STATE_HIGH
    elif g_state_c == STATE_HIGH and g_phase_cnt_c >= new_high_c:
        g_ch = 0
        g_cl = 1
        g_state_c = STATE_LOW
    g_period_c = new_period_c
    g_high_cnt_c = new_high_c

def TIM1_IRQHandler():
    global g_phase_cnt_a, g_phase_cnt_b, g_phase_cnt_c
    global g_ah, g_al, g_bh, g_bl, g_ch, g_cl
    global g_state_a, g_state_b, g_state_c
    
    # A相处理 - 用>=和状态检查防止错过切换点
    g_phase_cnt_a += 1
    if g_phase_cnt_a >= g_period_a:
        g_phase_cnt_a = 0
        g_al = 0
        g_ah = 1
        g_state_a = STATE_HIGH
    elif g_phase_cnt_a >= g_high_cnt_a and g_state_a == STATE_HIGH:
        g_ah = 0
        g_al = 1
        g_state_a = STATE_LOW
    
    # B相处理
    g_phase_cnt_b += 1
    if g_phase_cnt_b >= g_period_b:
        g_phase_cnt_b = 0
        g_bl = 0
        g_bh = 1
        g_state_b = STATE_HIGH
    elif g_phase_cnt_b >= g_high_cnt_b and g_state_b == STATE_HIGH:
        g_bh = 0
        g_bl = 1
        g_state_b = STATE_LOW
    
    # C相处理
    g_phase_cnt_c += 1
    if g_phase_cnt_c >= g_period_c:
        g_phase_cnt_c = 0
        g_cl = 0
        g_ch = 1
        g_state_c = STATE_HIGH
    elif g_phase_cnt_c >= g_high_cnt_c and g_state_c == STATE_HIGH:
        g_ch = 0
        g_cl = 1
        g_state_c = STATE_LOW

# Simulation
total_cycles = int(TIM1_IRQ_FREQ * SIMULATION_TIME)
tim7_period = int(TIM1_IRQ_FREQ / TIM7_IRQ_FREQ)

print("=" * 70)
print(f"TCM ZVS PWM Simulation - center={g_fsw_center/1000:.0f}kHz, duty={g_duty_cycle*100:.0f}%")
print("=" * 70)

# Data storage
time_data = []
ah_data = []
al_data = []
bh_data = []
bl_data = []
ch_data = []
cl_data = []
period_a_data = []
period_b_data = []
period_c_data = []
freq_a_data = []
freq_b_data = []
freq_c_data = []
high_cnt_a_data = []

# Initialize with TIM7 call
TIM7_IRQHandler()

# Simulation loop
tim7_counter = 0
for i in range(total_cycles):
    time_data.append(i / TIM1_IRQ_FREQ * 1000)
    
    tim7_counter += 1
    if tim7_counter >= tim7_period:
        tim7_counter = 0
        TIM7_IRQHandler()
    
    TIM1_IRQHandler()
    
    ah_data.append(g_ah)
    al_data.append(g_al)
    bh_data.append(g_bh)
    bl_data.append(g_bl)
    ch_data.append(g_ch)
    cl_data.append(g_cl)
    period_a_data.append(g_period_a)
    period_b_data.append(g_period_b)
    period_c_data.append(g_period_c)
    high_cnt_a_data.append(g_high_cnt_a)
    freq_a_data.append(TIM1_IRQ_FREQ / g_period_a / 1000)
    freq_b_data.append(TIM1_IRQ_FREQ / g_period_b / 1000)
    freq_c_data.append(TIM1_IRQ_FREQ / g_period_c / 1000)

# Convert to numpy arrays
time_data = np.array(time_data)
ah_data = np.array(ah_data)
al_data = np.array(al_data)
bh_data = np.array(bh_data)
bl_data = np.array(bl_data)
ch_data = np.array(ch_data)
cl_data = np.array(cl_data)
freq_a_data = np.array(freq_a_data)
freq_b_data = np.array(freq_b_data)
freq_c_data = np.array(freq_c_data)
high_cnt_a_data = np.array(high_cnt_a_data)
period_a_data = np.array(period_a_data)

print(f"Frequency range: {freq_a_data.min():.2f}kHz ~ {freq_a_data.max():.2f}kHz")

# 计算实际占空比
def calc_duty(ah, period_data, high_cnt_data):
    total_high = 0
    total_period = 0
    for i in range(len(ah)):
        total_high += ah[i]
        total_period += 1
    return total_high / total_period if total_period > 0 else 0

actual_duty = calc_duty(ah_data, period_a_data, high_cnt_a_data)
print(f"Expected duty: {g_duty_cycle*100:.1f}%")
print(f"Actual duty: {actual_duty*100:.1f}%")

# Detect wide pulses
print("\nDetecting abnormally wide pulses...")
wide_pulse_count = 0
for phase_name, ah, period_data, high_cnt_data in [('A', ah_data, period_a_data, high_cnt_a_data)]:
    high_start = None
    for i in range(1, len(ah)):
        if ah[i] == 1 and ah[i-1] == 0:
            high_start = i
        elif ah[i] == 0 and ah[i-1] == 1 and high_start is not None:
            width_us = i - high_start
            expected_high = high_cnt_data[high_start]
            if width_us > expected_high * 1.5:
                wide_pulse_count += 1
                print(f"  {phase_name}: Wide pulse at {time_data[high_start]:.2f}ms, width={width_us}us, expected={expected_high}us")

if wide_pulse_count == 0:
    print("  No abnormal pulses detected!")

# Plot
fig1, axes1 = plt.subplots(4, 1, figsize=(14, 12))
axes1[0].fill_between(time_data, 0, ah_data, alpha=0.7, color='red', label='AH')
axes1[0].fill_between(time_data, 0, al_data, alpha=0.7, color='blue', label='AL')
axes1[0].set_ylabel('A')
axes1[0].set_title(f'TCM ZVS PWM - duty={g_duty_cycle*100:.0f}%')
axes1[0].legend(loc='upper right')
axes1[0].set_ylim(-0.05, 1.05)
axes1[0].grid(True, alpha=0.3)

axes1[1].fill_between(time_data, 0, bh_data, alpha=0.7, color='red', label='BH')
axes1[1].fill_between(time_data, 0, bl_data, alpha=0.7, color='blue', label='BL')
axes1[1].set_ylabel('B')
axes1[1].legend(loc='upper right')
axes1[1].set_ylim(-0.05, 1.05)
axes1[1].grid(True, alpha=0.3)

axes1[2].fill_between(time_data, 0, ch_data, alpha=0.7, color='red', label='CH')
axes1[2].fill_between(time_data, 0, cl_data, alpha=0.7, color='blue', label='CL')
axes1[2].set_ylabel('C')
axes1[2].legend(loc='upper right')
axes1[2].set_ylim(-0.05, 1.05)
axes1[2].grid(True, alpha=0.3)

axes1[3].plot(time_data, freq_a_data, 'r-', label='A')
axes1[3].plot(time_data, freq_b_data, 'g-', label='B')
axes1[3].plot(time_data, freq_c_data, 'b-', label='C')
axes1[3].axhline(y=g_fsw_center/1000, color='gray', linestyle='--')
axes1[3].set_ylabel('Freq (kHz)')
axes1[3].set_xlabel('Time (ms)')
axes1[3].legend(loc='upper right')
axes1[3].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('tcm_zvs_simulation.png', dpi=150)

# Detail view
fig2, ax = plt.subplots(figsize=(14, 4))
start_idx = 5000
end_idx = 5500
t = time_data[start_idx:end_idx]
ax.step(t, ah_data[start_idx:end_idx], 'r-', where='post', label='AH')
ax.step(t, al_data[start_idx:end_idx], 'b-', where='post', label='AL')
ax.set_xlabel('Time (ms)')
ax.set_ylabel('GPIO')
ax.set_title('PWM Detail View')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('tcm_zvs_simulation_detail.png', dpi=150)

plt.show()

theoretical_min = (g_fsw_center - g_fsw_delta * g_voltage_output) / 1000
theoretical_max = (g_fsw_center + g_fsw_delta * g_voltage_output) / 1000
print(f"\nTheoretical freq: {theoretical_min:.2f}kHz ~ {theoretical_max:.2f}kHz")