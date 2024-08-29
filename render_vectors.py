import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

# Filepath to your data file
file_path = 'data.txt'

# Read data from file
def read_data(file_path):
    data = np.loadtxt(file_path, delimiter=',')
    return data

# Initialize plot
fig, ax = plt.subplots()

# Set axis limits
ax.set_xlim(-16000, 16000)
ax.set_ylim(-16000, 16000)

# Initialize vectors
accel_line, = ax.plot([], [], 'r-', lw=2, label='Acceleration Vector')
gyro_line, = ax.plot([], [], 'b-', lw=2, label='Gyroscope Vector')
accel_avg_line, = ax.plot([], [], 'g-', lw=2, label='Rolling Avg Acceleration')

# Initialize the vectors
accel_vec = np.array([0, 0])
gyro_vec = np.array([0, 0])

# Initialize deque to store the last 50 x and y values for the accelerometer
window_size = 50
accel_x_window = deque(maxlen=window_size)
accel_y_window = deque(maxlen=window_size)

def init():
    accel_line.set_data([], [])
    gyro_line.set_data([], [])
    accel_avg_line.set_data([], [])
    return accel_line, gyro_line, accel_avg_line

def animate(i):
    accel_vec = data[i, :2]  # Take only x and y for acceleration
    gyro_vec = data[i, 3:5]  # Take only x and y for gyroscope
    
    # Update the rolling window for accelerometer
    accel_x_window.append(accel_vec[0])
    accel_y_window.append(accel_vec[1])
    
    # Calculate rolling averages
    accel_x_avg = np.mean(accel_x_window)
    accel_y_avg = np.mean(accel_y_window)
    
    # Update the acceleration vector
    accel_line.set_data([0, accel_vec[0]], [0, accel_vec[1]])
    
    # Update the gyroscope vector
    gyro_line.set_data([0, gyro_vec[0]], [0, gyro_vec[1]])
    
    # Update the rolling average acceleration vector
    accel_avg_line.set_data([0, accel_x_avg], [0, accel_y_avg])
    
    return accel_line, gyro_line, accel_avg_line

# Read data
data = read_data(file_path)

# Create animation
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(data), interval=100, blit=True)

# Show the plot
plt.legend()
plt.show()
