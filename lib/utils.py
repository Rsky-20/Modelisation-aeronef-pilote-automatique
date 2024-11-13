import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import TransferFunction, bode
import matplotlib.pyplot as plt

# Constants
rho = 1.225  # Air density at sea level (kg/m^3)
g = 9.81     # Gravitational acceleration (m/s^2)

# Aircraft Parameters
mass = 1000   # Aircraft mass (kg)
S_ref = 16.2  # Reference wing area (m^2)
l_ref = 2.0   # Reference length (m)
I_xx = 10000  # Moment of inertia about x-axis (kg·m^2)
I_yy = 12000  # Moment of inertia about y-axis (kg·m^2)
I_zz = 8000   # Moment of inertia about z-axis (kg·m^2)

# Aerodynamic Coefficients
C_L_alpha = 5.7  # Lift curve slope
C_D_0 = 0.02     # Zero-lift drag coefficient
C_m_alpha = -1.2  # Pitching moment coefficient
C_L_delta = 0.3   # Lift control derivative
C_m_delta = -1.0  # Moment control derivative

# Flight Conditions
V = 100  # Airspeed (m/s)
alpha = 0.05  # Angle of attack (radians)

# Dynamic pressure
Q = 0.5 * rho * V**2

# Lift and drag
def aerodynamic_forces(alpha, delta):
    """Compute lift and drag forces."""
    lift = Q * S_ref * (C_L_alpha * alpha + C_L_delta * delta)
    drag = Q * S_ref * (C_D_0 + C_L_alpha * alpha**2)
    return lift, drag

# Longitudinal Dynamics
def longitudinal_dynamics(t, state, delta):
    """ODE for longitudinal dynamics."""
    u, w, q, theta, h = state  # State variables
    lift, drag = aerodynamic_forces(alpha, delta)
    u_dot = -drag / mass + q * w - g * np.sin(theta)
    w_dot = lift / mass - q * u + g * np.cos(theta)
    q_dot = (Q * S_ref * l_ref * (C_m_alpha * alpha + C_m_delta * delta)) / I_yy
    theta_dot = q
    h_dot = w * np.cos(theta) - u * np.sin(theta)
    return [u_dot, w_dot, q_dot, theta_dot, h_dot]

# Stability Analysis
def characteristic_equation(Z_alpha, m_q, m_alpha):
    """Compute characteristic equation roots for stability."""
    a = 1
    b = Z_alpha - m_q
    c = -(m_alpha + Z_alpha * m_q)
    roots = np.roots([a, b, c])
    return roots

# Transfer Functions
def pitch_transfer_function():
    """Define transfer function for pitch dynamics."""
    T_alpha = -C_m_delta / (C_m_alpha * C_L_delta - C_m_delta * C_L_alpha)
    omega_z = np.sqrt(-C_m_delta * (C_m_alpha * C_L_delta - C_m_delta * C_L_alpha) / C_L_delta)
    xi = -0.5 * (C_L_alpha + C_m_alpha) / omega_z
    num = [T_alpha, 1]
    den = [1, 2 * xi * omega_z, omega_z**2]
    return TransferFunction(num, den)

# Simulation
initial_state = [V, 0, 0, 0, 0]  # [u, w, q, theta, h]
t_span = (0, 10)
t_eval = np.linspace(*t_span, 500)

# Elevator deflection (control input)
delta_e = 0.01  # Elevator deflection (radians)
sol = solve_ivp(longitudinal_dynamics, t_span, initial_state, args=(delta_e,), t_eval=t_eval)

# Plot results
plt.figure()
plt.plot(sol.t, sol.y[0], label='Forward velocity (u)')
plt.plot(sol.t, sol.y[1], label='Vertical velocity (w)')
plt.plot(sol.t, sol.y[3], label='Pitch angle (theta)')
plt.xlabel('Time (s)')
plt.ylabel('State Variables')
plt.legend()
plt.title('Longitudinal Dynamics')
plt.grid()
plt.show()

# Stability Analysis Example
Z_alpha, m_q, m_alpha = -0.5, -1.2, -0.8
roots = characteristic_equation(Z_alpha, m_q, m_alpha)
print("Characteristic equation roots:", roots)

# Pitch Transfer Function Example
tf = pitch_transfer_function()
w, mag, phase = bode(tf)

# Plot Bode Diagram
plt.figure()
plt.subplot(2, 1, 1)
plt.semilogx(w, mag)
plt.title('Bode Diagram')
plt.ylabel('Magnitude (dB)')
plt.grid()

plt.subplot(2, 1, 2)
plt.semilogx(w, phase)
plt.xlabel('Frequency (rad/s)')
plt.ylabel('Phase (degrees)')
plt.grid()
plt.show()


import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import TransferFunction, bode, lti, step
import matplotlib.pyplot as plt
from control import ss, feedback, damp, tf, step_response

# Constants
rho = 1.225  # Air density at sea level (kg/m^3)
g = 9.81     # Gravitational acceleration (m/s^2)

# Aircraft Characteristics
mass = 8400  # Aircraft mass (kg)
S = 34       # Reference wing area (m^2)
lt = 5.24    # Reference length (m)
c = 0.52     # Center of gravity (as a percentage of total length)

# Aerodynamic Coefficients
Cx0 = 0.03  # Drag coefficient at zero angle of attack
Cz_alpha = 3.5  # Lift gradient coefficient w.r.t. angle of attack
Cz_delta = 1.2  # Lift gradient coefficient w.r.t. elevator deflection
Cm_q = -0.5     # Pitch damping coefficient
Cm_delta = -1.1 # Moment coefficient w.r.t. elevator deflection

# Equilibrium Conditions
def equilibrium_conditions(alpha, delta_m):
    """Compute aerodynamic forces and trim conditions."""
    Q = 0.5 * rho * V**2
    Cz = Cz_alpha * alpha + Cz_delta * delta_m
    Cx = Cx0 + Cz**2 * 0.2  # Assuming polar coefficient k = 0.2
    L = Q * S * Cz
    D = Q * S * Cx
    return L, D

# State-Space Representation
A = np.array([[-0.02, -0.05, 0, 0],
              [0, 0, 1, 0],
              [0, -1.2, -0.3, 0.5],
              [0, 0, 0, -1.1]])
B = np.array([[0.01],
              [0],
              [1.2],
              [0.8]])
C = np.eye(4)
D = np.zeros((4, 1))

system = ss(A, B, C, D)

# Feedback Loops
def design_feedback_gain(system, desired_damping_ratio, natural_frequency):
    """Design feedback gain for a desired damping ratio and natural frequency."""
    poles, damping_ratios, frequencies = damp(system)
    gain = desired_damping_ratio * natural_frequency / max(poles.real)
    return gain

# Step Response
def plot_step_response(system, title):
    """Plot step response of the system."""
    T, yout = step_response(system)
    plt.figure()
    plt.plot(T, yout)
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel('Response')
    plt.grid()
    plt.show()

# Open-Loop Analysis
print("Open-Loop Poles and Damping Ratios:")
damp(system)

# q Feedback Loop
Kq = -0.1
q_feedback_system = feedback(Kq * system, 1)
plot_step_response(q_feedback_system, "Step Response with q Feedback Loop")

# γ Feedback Loop
Kgamma = -0.05
gamma_feedback_system = feedback(Kgamma * q_feedback_system, 1)
plot_step_response(gamma_feedback_system, "Step Response with γ Feedback Loop")

# z Feedback Loop
Kz = -0.02
z_feedback_system = feedback(Kz * gamma_feedback_system, 1)
plot_step_response(z_feedback_system, "Step Response with z Feedback Loop")

# Saturation Handling
def compute_alpha_max(alpha_eq, delta_nz):
    """Compute maximum angle of attack based on load factor."""
    alpha_max = alpha_eq + (alpha_eq - 0.02) * delta_nz  # Assuming α0 = 0.02 rad
    return alpha_max

alpha_eq = 0.05  # Trim angle of attack (rad)
delta_nz = 3.2  # Load factor increment (g)
alpha_max = compute_alpha_max(alpha_eq, delta_nz)
print(f"Maximum angle of attack (αmax): {alpha_max:.3f} rad")

