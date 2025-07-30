import numpy as np
from scipy.integrate import cumulative_trapezoid

def simulate_distance(a_val, freq_hz, duration_sec):
    dt = 1.0 / freq_hz
    num_samples = int(freq_hz * duration_sec)
    a = np.full(num_samples, a_val)
    
    # First integration: a → v
    v = cumulative_trapezoid(a, dx=dt, initial=0)

    # Second integration: v → x
    x = cumulative_trapezoid(v, dx=dt, initial=0)
    
    return round(x[-1], 4)

# Try different frequencies
for freq in [2, 5, 10, 50, 100]:
    dist = simulate_distance(a_val=2.0, freq_hz=freq, duration_sec=15)
    print(f"Freq: {freq} Hz -> Distance: {dist} m")
