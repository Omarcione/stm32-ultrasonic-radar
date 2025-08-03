import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time
import serial
from collections import deque

# === Try serial ===
try:
    ser = serial.Serial('COM3', 115200, timeout=0.1)
    print("Serial connected.")
except Exception as e:
    ser = None
    print(f"WARNING: Serial not connected. Simulation mode enabled. {e}")

# === Radar config ===
MAX_DISTANCE_CM = 20
RADAR_RADIUS = 200  # For visualization scale

# === Plot setup ===
fig = plt.figure(figsize=(8, 8), facecolor='black')
ax = plt.subplot(111, polar=True)
ax.set_facecolor('black')
plt.tight_layout()

# Radar orientation: 180° at top, moving counterclockwise to 0°
ax.set_theta_zero_location('E')
ax.set_theta_direction(1)  # Change to 1 for counterclockwise
ax.set_ylim(0, RADAR_RADIUS)
ax.set_thetamin(180)  # Switch from 0 to 180
ax.set_thetamax(0)    # Switch from 180 to 0

# Grid styling
ax.set_xticks(np.radians(np.linspace(180, 0, 7)))  # Reverse the linspace range
ax.set_xticklabels([f'{int(a)}°' for a in np.linspace(180, 0, 7)], color='white')  # Reverse the linspace range
ax.set_yticks(np.linspace(0, RADAR_RADIUS, 5))
ax.set_yticklabels([f'{int(r * MAX_DISTANCE_CM / RADAR_RADIUS)} cm' for r in np.linspace(0, RADAR_RADIUS, 5)], color='white')
ax.grid(True, color='white', linestyle='dotted', linewidth=0.7)
ax.plot(np.radians([180, 0]), [RADAR_RADIUS, RADAR_RADIUS], color='white', linewidth=2)  # Switch angles here too

# Radar sweep and dots
sweep_line, = ax.plot([], [], color='green', linewidth=2)
dots = []
dot_data = deque(maxlen=50)  # Store (theta, distance, timestamp) tuples

# Object data
current_angle = 0
current_distance = -1
last_received_angle = 0  # Add this line to track received angle

#update the display
def update(frame):
    global current_angle, current_distance, dots, last_received_angle
    current_time = time.time()

    # Read serial if connected
    if ser and ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if line:
                angle_str, distance_str = line.split(',')
                angle = float(angle_str.strip())
                last_received_angle = angle  # Update sweep angle to received angle
                
                distance = float(distance_str.strip())
                if distance > 0:  # Only store dots for valid detections
                    theta = np.radians(angle)
                    scaled_distance = (distance / MAX_DISTANCE_CM) * RADAR_RADIUS
                    dot_data.append((theta, scaled_distance, current_time))
        except:
            pass  # ignore parsing errors
    else:
        # Demo mode - sweep from 180 to 0 degrees
        last_received_angle -= 2  # Adjust speed by changing this value
        if last_received_angle < 0:
            last_received_angle = 180

    # Update sweep line
    theta = np.radians(last_received_angle)
    sweep_line.set_data([theta, theta], [0, RADAR_RADIUS])

    # Remove old dots
    for dot in dots:
        dot.remove()
    dots.clear()

    # Draw all dots less than 1 second old
    for theta, distance, timestamp in dot_data:
        if current_time - timestamp <= 1.0:
            dot = ax.plot(theta, distance, 'ro', markersize=10)[0]
            dots.append(dot)
    
    # Clean up old data points
    while dot_data and current_time - dot_data[0][2] > 1.0:
        dot_data.popleft()

    return [sweep_line] + dots

# Animate
ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()

if ser:
    ser.close()
