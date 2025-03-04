import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Simulate radar data
def simulate_radar_data(num_samples, num_chirps, num_targets):
    data = np.random.randn(num_chirps, num_samples) + 1j * np.random.randn(num_chirps, num_samples)
    for _ in range(num_targets):
        range_idx = np.random.randint(0, num_samples)
        doppler_idx = np.random.randint(0, num_chirps)
        data[doppler_idx, range_idx] += 100  # Add a strong reflection
    return data

# Range-Doppler processing
def range_doppler_processing(data):
    range_fft = np.fft.fft(data, axis=1)
    doppler_fft = np.fft.fft(range_fft, axis=0)
    return np.abs(doppler_fft)

# Object detect
c = 3e8  # Speed of light (m/s)
fc = 77e9  # Carrier frequency (77 GHz)
B = 4e9  # Bandwidth (4 GHz)
Tc = 1e-4  # Chirp duration (100 µs)
Nr = 256  # Number of range samples
Nd = 64  # Number of Doppler samples

# Target parameters
range_target = 50  # Target at 50 meters
velocity_target = 20  # Target moving at 20 m/s

# Generate simulated radar data
t = np.linspace(0, Tc, Nr)
f = np.linspace(0, B, Nr)

range_data = np.zeros((Nd, Nr), dtype=complex)
for n in range(Nd):
    tau = 2 * (range_target + n * Tc * velocity_target) / c
    phase = 2 * np.pi * (fc * tau + 0.5 * B * tau**2 / Tc - f * tau)
    range_data[n, :] = np.exp(1j * phase)

# Add some noise
noise = np.random.normal(0, 0.1, (Nd, Nr)) + 1j * np.random.normal(0, 0.1, (Nd, Nr))
range_data += noise

# Main processing pipeline
num_samples = 256
num_chirps = 128
num_targets = 3

# First one is random noise
# radar_data = simulate_radar_data(num_samples, num_chirps, num_targets) 
range_doppler_map = range_doppler_processing(range_data)
# detected_targets = cfar_detection(range_doppler_map, guard_cells=1, training_cells=2)


# Visualization
plt.imshow(np.log10(range_doppler_map), aspect='auto', cmap='jet')
plt.title('Range-Doppler Map')
plt.xlabel('Range')
plt.ylabel('Doppler')
plt.show()