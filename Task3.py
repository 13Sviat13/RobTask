import numpy as np
import matplotlib.pyplot as plt

# PID parameters
Kp_values = [0.0, 0.1]
Ki_values = [0.0, 0.1, 0.5]
Kd_values = [0.0, 0.1, 0.5]

# Simulation parameters
time_steps = 100
y0 = 1.0

t = np.arange(time_steps)

# Function to simulate the PID controller
def pid_controller(Kp, Ki, Kd, y0, time_steps):
   y = np.zeros(time_steps)
   y[0] = y0
   integral = 0.0
   for i in range(1, time_steps):
       error = -y[i-1]
       integral += error
       derivative = y[i-1] - y[i-2] if i > 1 else 0
       y[i] = Kp * error + Ki * integral + Kd * derivative
   return y

# Plot results for different combinations of Kp, Ki, Kd
fig, axs = plt.subplots(len(Ki_values), len(Kd_values), figsize=(15, 10))
fig.suptitle("PID Controller Response with Varying Parameters")

for i, Ki in enumerate(Ki_values):
   for j, Kd in enumerate(Kd_values):
       axs[i, j].set_title(f'Ki = {Ki}, Kd = {Kd}')
       for Kp in Kp_values:
           y = pid_controller(Kp, Ki, Kd, y0, time_steps)
           axs[i, j].plot(t, y, label=f'Kp = {Kp}')
       axs[i, j].legend()
       axs[i, j].set_xlabel('Time')
       axs[i, j].set_ylabel('y(t)')

plt.tight_layout()
plt.subplots_adjust(top=0.9)
plt.show()
