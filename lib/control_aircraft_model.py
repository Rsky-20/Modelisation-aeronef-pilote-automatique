import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import TransferFunction, bode, ss2tf, step
import matplotlib.pyplot as plt
from control import *
import control.matlab
import math
import os
import sys

try:
    from sisopy31 import *
    import sisopy31 as sisopy31
except:
    from lib.sisopy31 import *
    import lib.sisopy31 as sisopy31

try:
    import atm_std
except:
    import lib.atm_std as atm_std
    
import time

OUTPUT_FIG_PATH = "D:\\IPSA\\Aero5\\2.12_au511_Modelisation_aeronef_pilote_automatique\\Modelisation-aeronef-pilote-automatique\\data\\fig"

if not os.path.exists(OUTPUT_FIG_PATH):
    os.makedirs(OUTPUT_FIG_PATH)  # Crée les répertoires manquants
            
class ControlAircraftModel:
    def __init__(self, name='MIRAGE III class'):
        self.name = name
        self.save_fig = False
        # Constants
        self.g = 9.81     # Gravitational acceleration (m/s^2)
        self.tau = 0.75   # time constant for transfert function (s)
        self.r_g = 2.65 #m

        # Flight condition 
        self.mach_speed = 1.88  # Aircraft Mach speed (relative to speed of sound)
        self.altitude = self.ft_to_m(22265)   # Aircraft cruise altitude (ft)
        
        if self.mach_speed == 1.88 and self.altitude == self.ft_to_m(22265):
            _, self.rho, self.V_s =  atm_std.get_cte_atm(self.altitude) #hgeo = , roh = sensité de l'air, V_s = vitesse du son 
            self.rho = float(self.rho)
            self.V_s = float(self.V_s)
            self.V_a = self.mach_speed * self.V_s #aerodynamic speed of the aircraft (m/s)

            # Aircraft Characteristics
            self.mass = 8400        # Aircraft mass (kg)
            self.S = 34             # Reference wing area (m^2)
            self.l_ref = 5.24          # Reference length (m)
            self.lt = (3/2) * self.l_ref
            self.c = 0.52           # Center of gravity (as a percentage of total length)
            self.I_yy = self.mass * pow(self.r_g, 2)       # Moment of inertia about the y-axis (kg·m^2)

            # Aerodynamic Coefficients
            self.C_x_0 = 0.029         # Drag coefficient at zero angle of attack
            self.C_z_alpha = 2.124    # Lift gradient coefficient w.r.t. angle of attack
            self.C_z_delta_m = 0.316    # Lift gradient coefficient w.r.t. elevator deflection
            self.delta_m_0 = -0.0017   # Equilibrium fin deflection for null lift 
            self.alpha_0 = 0.011    # Incidence for null lift and null fin deflection (rad)
            self.f = 0.608          # Aerodynamique center of body and wings
            self.f_delta = 0.9      # Aerodynamique center of fins (pitch axis)
            self.k = 0.53           # Polar coefficient 
            self.C_m_q = -0.266        # Pitch damping coefficient (s/rad)
            self.C_m_delta = -1.1    # Moment coefficient w.r.t. elevator deflection
            self.F_delta = self.f_delta * self.lt
            self.F = self.f * self.lt
            
            self.Q = 0.5 * self.rho * pow(self.V_a, 2)

            #X,Y bras de levier distance entre F et G :  X_g - X_f
            self.X =  -(self.f * self.lt - self.c * self.lt)
            self.Y =  -(self.f_delta - self.c) * self.lt

            self.V_eq = self.V_a #car on est pas loin du point d'équilibre 
        else:
            print("[WARNING] - Altitude and Mach speed are differente from what we use in this script ! Correct Value before continue")
        
        
        self.X_state = None
        self.Y_state = None
        self.matrix_A = np.array([])  # Tableau vide
        self.matrix_B = np.array([])  # Tableau vide
        self.matrix_C = np.array([])  # Tableau vide
        self.matrix_D = np.array([])  # Tableau vide
        self.gamma_eq = 0
        
        # Initialisation des variables mentionnées
        self.C_z_eq = 0              # Coefficient aérodynamique initial
        self.C_x_eq = 0              # Coefficient aérodynamique initial
        self.C_x_delta_m = 0         # Coefficient de contrôle initial
        self.delta_m_eq = 0          # Déviation initiale
        self.alpha_eq_list = []      # Liste vide pour les angles d'équilibre
        self.F_p_x_eq = 0            # Force de propulsion initiale

        self.Z_v = 0                 # Variable aérodynamique initiale
        self.alpha_eq = 0            # Angle d'équilibre initial
        self.F_eq = 0                # Force d'équilibre initiale
        self.C_z = self.C_z_eq       # Initialisation égale au coefficient déjà défini
        self.C_x_alpha = 0           # Coefficient aérodynamique
        self.C_m_alpha = 0           # Moment aérodynamique
        self.Z_alpha = 0             # Coefficient aérodynamique angulaire
        self.Z_delta_m = 0           # Coefficient de déviation
        self.m_alpha = 0             # Moment angulaire
        self.m_q = 0                 # Moment d'inertie
        self.C_m_delta_m = 0         # Moment aérodynamique
        self.m_delta_m = 0           # Moment d'inertie dû à la déviation
        self.X_v = 0                 # Coefficient aérodynamique
        self.X_gamma = 0             # Coefficient lié à l'angle de montée
        self.X_alpha = 0             # Coefficient lié à l'angle d'attaque
        

        self.openloop_sys = None     
        self.gain_Kr = 1
        self.openloop_eigenA = None
        self.openloop_eigen_list = []
        self.openloop_time, self.openloop_response = None, None
        
        self.sp_eigenA, self.phu_eigenA = None, None
        self.sp_eigen_list, self.phu_eigen_list = [], []
        self.fig_phu, self.fig_sp, self.ax_phu, self.ax_sp = None, None, None, None
        self.gain_k_sp_alpha, self.gain_k_sp_q,self.gain_k_phu_v, self.gain_k_phu_g = 1, 1, 1,1

    
    #############################################
    #   Aircraft Longitudinal Dynamics
    #############################################
    
    def compute_F_eq(self, rho, V_eq, S, C_x_eq, alpha_eq):
        return (0.5 * rho * pow(V_eq, 2) * S * C_x_eq) / (np.cos(alpha_eq))
    
    
    def compute_X_v(self, Q, S, C_x_eq, m, V_eq):
        return (2 * Q * S * C_x_eq) / (m * V_eq)
    
    
    def compute_X_alpha(self, F_eq, m, V_eq, alpha_eq, Q, S, C_x_alpha):
        return ((F_eq)/(m * V_eq)) * np.sin(alpha_eq) + (Q * S * C_x_alpha) / (m * V_eq)
    
    
    def compute_X_gamma(self, g_0, gamma_eq, V_eq):
        return (g_0 * np.cos(gamma_eq))/V_eq
    
    
    def compute_X_delta_m(self, Q, S, C_x_delta_m, m, V_eq):
        return (Q * S * C_x_delta_m) / (m * V_eq)
    
    
    def compute_X_tau(self, F_tau, alpha_eq, m, V_eq):
        return -(F_tau * np.cos(alpha_eq)) / (m * V_eq)
    
    
    def compute_m_v(self):
        return 0
    
    
    def compute_m_alpha(self, Q, S, l_ref, C_m_alpha, I_yy):
        return (Q * S * l_ref * C_m_alpha) / I_yy
    
    
    def compute_m_q(self, Q, S, l_ref, C_m_q, V_eq, I_yy):
        return (Q * S * pow(l_ref, 2) * C_m_q) / (V_eq * I_yy)
    
    
    def compute_m_delta_m(self, Q, S, l_ref, C_m_delta_m, I_yy):
        return (Q * S * l_ref * C_m_delta_m) / I_yy
    
    
    def compute_Z_V(self, Q, S, C_z_eq, m, V_eq):
        return (2 * Q * S * C_z_eq) / (m * V_eq)
    
    
    def compute_Z_V_(self, g_0, V_eq):
        return (2 * g_0) / V_eq
    
    
    def compute_Z_alpha(self, F_eq, m, V_eq, alpha_eq, Q, S, C_z_alpha):
        return (F_eq / (m * V_eq)) * np.cos(alpha_eq) + (Q * S * C_z_alpha) / (m * V_eq)
    
    
    def compute_Z_gamma(self, g_0, gamma_eq, V_eq):
        return (g_0 * np.sin(gamma_eq)) / V_eq
    
    
    def compute_Z_delta_m(self, Q, S, C_z_delta_m, m, V_eq):
        return (Q * S * C_z_delta_m) / (m * V_eq)  
    
    
    def compute_Z_tau(self, F_tau, alpha_eq, m, V_eq):
        return (F_tau * np.sin(alpha_eq)) / (m * V_eq)
    
    
    def compute_C_m_delta_m(self, Y, l_ref, C_x_delta_m, alpha_eq, C_z_delta_m):
        return (Y/l_ref) * (C_x_delta_m * np.sin(alpha_eq) + C_z_delta_m * np.cos(alpha_eq))
    
    
    def compute_C_x_alpha(self, k, C_z, C_z_alpha):
        return 2 * k * C_z * C_z_alpha
    
    
    def compute_C_m_alpha(self, X, l_ref, C_x_alpha, alpha_eq, C_z_alpha):
        #(X/lref)*(Cxa*np.sin(aeq) + Cza*np.cos(aeq))
        return (X / l_ref) * (C_x_alpha * np.sin(alpha_eq) + C_z_alpha * np.cos(alpha_eq))
    
    
    def compute_A_array(self, X_V, X_gamma, X_alpha, Z_V, Z_alpha, m_alpha, m_q, V_eq):
        self.matrix_A = np.array(
            [[-X_V, -X_gamma, -X_alpha, 0, 0, 0],
             [Z_V, 0, Z_alpha, 0, 0, 0],
             [-Z_V, 0, -Z_alpha, 1, 0, 0],
             [0, 0, m_alpha, m_q, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, V_eq, 0, 0, 0, 0]])

    
    def compute_B_array(self, Z_delta_m, m_delta_m):
        self.matrix_B = np.array([0, Z_delta_m, -Z_delta_m, m_delta_m, 0, 0]).T
        
        
    def compute_C_array(self):
        num_states = 6  # Number of states (V, gamma, alpha, q, theta, z)
        self.matrix_C = np.eye(num_states)  # Identity matrix


    def compute_D_array(self):
        num_outputs = 6  # Number of outputs (same as the number of states)
        num_inputs = 1   # One input (delta_m)
        self.matrix_D = np.zeros((num_outputs, num_inputs))  # Zero matrix


    def compute_X_p(self, A, X, B, U):
        return np.dot(A, X) + np.dot(B, U)
       
        
    def equilibrium_point(self, F_p_x_eq=0, alpha_eq_0=0, epsilon=1e-6):

        # Initialisation des variables
        C_z_eq = 0
        alpha_eq_prec = alpha_eq_0
        C_x_eq = 0
        C_x_delta_m = 0
        delta_m_eq = 0
        F_p_x_eq = 0
        alpha_eq = 1
        alpha_eq_list = [alpha_eq_0]  # Initialisation de la liste alpha_eq_list avec alpha_eq_0

        # Boucle while avec condition correcte
        while abs(alpha_eq_prec - alpha_eq) > epsilon:
            C_z_eq = (1 / (self.Q * self.S)) * (self.mass * self.g - F_p_x_eq * np.sin(alpha_eq))
            C_x_eq = self.C_x_0 + self.k * (C_z_eq ** 2)  # Correction de l'exponentiation
            alpha_eq_prec = alpha_eq
            C_x_delta_m = 2 * self.k * C_z_eq * self.C_z_delta_m
            delta_m_eq = self.delta_m_0 - ((C_x_eq * np.sin(alpha_eq) + C_z_eq * np.cos(alpha_eq)) /
                                    (C_x_delta_m * np.sin(alpha_eq) + self.C_z_delta_m * np.cos(alpha_eq))) * (self.X / (self.Y - self.X))
            F_p_x_eq = (self.Q * self.S * C_x_eq) / np.cos(alpha_eq)
            alpha_eq = self.alpha_0 + C_z_eq / self.C_z_alpha - self.C_z_delta_m * delta_m_eq / self.C_z_alpha
            
            alpha_eq_list.append(alpha_eq)  # Mise à jour de la liste alpha_eq_list
        return C_z_eq, C_x_eq, C_x_delta_m, delta_m_eq, alpha_eq_list, F_p_x_eq
        
        
    def compute_dynamic_matrix(self):
        #if self.X_v is not None and self.X_gamma is not None and self.X_alpha is not None and self.Z_v is not None and self.Z_alpha is not None and self.m_alpha is not None and self.m_q is not None and self.V_eq is not None:
        self.compute_A_array(self.X_v, self.X_gamma, self.X_alpha, self.Z_v, self.Z_alpha, self.m_alpha, self.m_q, self.V_eq)
        self.compute_C_array()


    def compute_control_matrix(self):
        #if self.Z_delta_m is not None and self.m_delta_m is not None:
        self.compute_B_array(self.Z_delta_m, self.m_delta_m)
        self.compute_D_array()

    
    def equilibrium_conditions(self):
        """Compute aerodynamic forces and trim conditions."""
        self.C_z_eq, self.C_x_eq, self.C_x_delta_m, self.delta_m_eq, self.alpha_eq_list, self.F_p_x_eq = self.equilibrium_point()
        
        self.Z_v = self.compute_Z_V_(self.g, self.V_eq)
        self.alpha_eq = self.alpha_eq_list[-1]
        self.F_eq = self.compute_F_eq(self.rho, self.V_eq, self.S, self.C_x_eq, self.alpha_eq)
        self.C_z = self.C_z_eq
        self.C_x_alpha = self.compute_C_x_alpha(self.k, self.C_z, self.C_z_alpha)
        self.C_m_alpha = self.compute_C_m_alpha(self.X,self.l_ref, self.C_x_alpha, self.alpha_eq, self.C_z_alpha)
        self.Z_alpha = self.compute_Z_alpha(self.F_eq, self.mass, self.V_eq, self.alpha_eq, self.Q, self.S, self.C_z_alpha)
        self.Z_delta_m = self.compute_Z_delta_m(self.Q, self.S, self.C_z_delta_m, self.mass, self.V_eq)
        self.m_alpha = self.compute_m_alpha(self.Q, self.S, self.l_ref, self.C_m_alpha, self.I_yy)
        self.m_q = self.compute_m_q(self.Q, self.S, self.l_ref, self.C_m_q, self.V_eq, self.I_yy)
        self.C_m_delta_m = self.compute_C_m_delta_m(self.Y, self.l_ref, self.C_x_delta_m, self.alpha_eq, self.C_z_delta_m)
        self.m_delta_m = self.compute_m_delta_m(self.Q, self.S, self.l_ref, self.C_m_delta_m, self.I_yy)
        self.X_v = self.compute_X_v(self.Q, self.S, self.C_x_eq, self.mass, self.V_eq)
        self.X_gamma = self.compute_X_gamma(self.g, self.gamma_eq, self.V_eq)
        self.X_alpha = self.compute_X_alpha(self.F_eq,self.mass, self.V_eq, self.alpha_eq, self.Q, self.S, self.C_x_alpha)
        
    
    def K1(self, V_eq, m_alpha, Z_delta_m, m_delta_m, Z_alpha, m_q):
        return (-V_eq * (-m_alpha * Z_delta_m + m_delta_m * Z_alpha)) / (-m_q * Z_alpha - m_alpha)
    
    
    def omega(self, Z_delta_m, m_alpha, Z_alpha, m_delta_m):
        return math.sqrt(-Z_delta_m * (-m_alpha * Z_delta_m + m_delta_m * Z_alpha)) / Z_delta_m
    
    
    def tau_z(self, Z_delta_m, m_q, m_alpha, m_delta_m, Z_alpha):
        return -((Z_delta_m * m_q) / (-m_alpha * Z_delta_m + m_delta_m * Z_alpha))
    
    
    def compute_phugoid_mode_matrix(self):
        self.matrix_A_phugoid = self.matrix_A[0:2,0:2]
        self.matrix_B_phugoid = self.matrix_B[0:2]
        self.C_phugoid_V = np.array([1,0])
        self.C_phugoid_gamma = np.array([0,1])
        
        
    def compute_short_period_mode_matrix(self):
        self.matrix_A_short_period=self.matrix_A[2:4,2:4]
        self.matrix_B_short_period=self.matrix_B[2:4]
        self.C_short_period_alpha = np.array([1,0])
        self.C_short_period_q = np.array([0,1])

    
    def open_loop(self):
        # Calcul des valeurs propres
        
        self.openloop_eigenA = np.linalg.eigvals(self.matrix_A)
        for pole in self.openloop_eigenA:
            sigma = pole.real  # Partie réelle
            omega = pole.imag  # Partie imaginaire
            omega_n = np.sqrt(sigma**2 + omega**2)
            zeta = -sigma / omega_n if omega_n != 0 else 0  # Évite la division par zéro
            self.openloop_eigen_list.append({'eigen' : pole, 'wn' : round(omega_n,4), 'Xi' : round(zeta, 4)})

        self.openloop_sys = control.ss(self.matrix_A, self.matrix_B, self.matrix_C, self.matrix_D)
        self.wn, self.damping, self.eigenvalues = control.damp(self.openloop_sys)


    def compute_system(self, matrix_A, matrix_B, C_vector, mode='tf'):
        ss_system = control.ss(matrix_A, matrix_B, C_vector.T, 0)
        wn, damping, eigenvalues = control.damp(ss_system)
        if mode == 'tf':
            tf_system = control.ss2tf(ss_system)
        else:
            tf_system = control.ss2tf(ss_system)
        return ss_system, wn, damping, eigenvalues, tf_system

    def step_response_graph(self, ax, fig, T1, Y1, T2, Y2, labels, title, xlabel, ylabel, fig_name):
        ax.plot(T1, Y1, "b", label=labels[0], lw=2)
        ax.plot(T2, Y2, "r", label=labels[1], lw=2)
        ax.plot([0, T1[-1]], [Y1[-1], Y1[-1]], 'k--', lw=1)
        ax.plot([0, T1[-1]], [1.05 * Y1[-1], 1.05 * Y1[-1]], 'k--', lw=1)
        ax.plot([0, T1[-1]], [0.95 * Y1[-1], 0.95 * Y1[-1]], 'k--', lw=1)
        ax.plot([0, T2[-1]], [Y2[-1], Y2[-1]], 'k--', lw=1)
        ax.plot([0, T2[-1]], [1.05 * Y2[-1], 1.05 * Y2[-1]], 'k--', lw=1)
        ax.plot([0, T2[-1]], [0.95 * Y2[-1], 0.95 * Y2[-1]], 'k--', lw=1)
        ax.minorticks_on()
        ax.grid(True)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.legend()
        
        ax.figure.canvas.manager.set_window_title(fig_name)
        if self.save_fig:
            ax.figure.savefig(os.path.join(OUTPUT_FIG_PATH, f'{fig_name}.png'))
        else:
            plt.show()



    def compute_transient_phase(self):
        # Calcul des matrices pour les deux modes
        self.compute_short_period_mode_matrix()
        self.compute_phugoid_mode_matrix()

        # Vérifiez les matrices nécessaires
        assert hasattr(self, "matrix_A_short_period"), "Short period matrix A is not defined."
        assert hasattr(self, "matrix_B_short_period"), "Short period matrix B is not defined."
        assert hasattr(self, "matrix_A_phugoid"), "Phugoid matrix A is not defined."

        # Calcul des systèmes pour short period
        self.ss_sp_alpha, self.sp_wn_alpha, self.sp_damping_alpha, self.sp_eigenvalues_alpha, self.tf_sp_alpha = \
            self.compute_system(self.matrix_A_short_period, self.matrix_B_short_period, self.C_short_period_alpha)
        self.ss_sp_q, self.sp_wn_q, self.sp_damping_q, self.sp_eigenvalues_q, self.tf_sp_q = \
            self.compute_system(self.matrix_A_short_period, self.matrix_B_short_period, self.C_short_period_q, mode='ss2tf')

        # Calcul des systèmes pour phugoïde
        self.ss_phu_v, self.phu_wn_v, self.phu_damping_v, self.phu_eigenvalues_v, self.tf_phu_v = \
            self.compute_system(self.matrix_A_phugoid, self.matrix_B_phugoid, self.C_phugoid_V)
        self.ss_phu_g, self.phu_wn_g, self.phu_damping_g, self.phu_eigenvalues_g, self.tf_phu_g = \
            self.compute_system(self.matrix_A_phugoid, self.matrix_B_phugoid, self.C_phugoid_gamma, mode='ss2tf')

        #self.gain_k_sp_alpha = sisopy31.sisotool(1*minreal(self.ss_sp_alpha))
        #self.gain_k_sp_q = sisopy31.sisotool(1*minreal(self.ss_sp_q))
        #self.gain_k_phu_v = sisopy31.sisotool(1*minreal(self.ss_phu_v))
        #self.gain_k_phu_g = sisopy31.sisotool(1*minreal(self.ss_phu_g))
        
        # Tracé des réponses en échelon pour short period
        self.fig_sp, self.ax_sp = plt.subplots()
        Ya, Ta = control.matlab.step(self.tf_sp_alpha, arange(0, 10, 0.01))
        Yq, Tq = control.matlab.step(self.tf_sp_q, arange(0, 10, 0.01))
        self.step_response_graph(
            self.ax_sp, self.fig_sp,
            Ta, Ya, Tq, Yq,
            labels=[r"$\alpha/\delta_m$", r"$q/\delta_m$"],
            title=r"Step response $\alpha/\delta_m$ et $q/\delta_m$",
            xlabel="Time (s)",
            ylabel=r"$\alpha$ (rad) & $q$ (rad/s)",
            fig_name="Short_Period_Step_Response"
        )

        # Tracé des réponses en échelon pour phugoïde
        self.fig_phu, self.ax_phu = plt.subplots()
        Yv, Tv = control.matlab.step(self.tf_phu_v, arange(0, 700, 0.1))
        Yg, Tg = control.matlab.step(self.tf_phu_g, arange(0, 700, 0.1))
        self.step_response_graph(
            self.ax_phu, self.fig_phu,
            Tv, Yv, Tg, Yg,
            labels=[r"$V/\delta_m$", r"$\gamma/\delta_m$"],
            title=r"Step response $V/\delta_m$ et $\gamma/\delta_m$",
            xlabel="Time (s)",
            ylabel=r"$V$ (m/s) & $\gamma$ (rad)",
            fig_name="Phugoid_Step_Response"
        )
        
        _, _, self.Tsa = step_info(Ta, Ya)
        _, _, self.Tsq = step_info(Tq, Yq)
        
        _, _, self.Tsv = step_info(Tv, Yv)
        _, _, self.Tsg = step_info(Tg, Yg)        
        
    
    def compute_feedback_loop(self, label: str):
        def plot_step_response(ax, T1, Y1, labels, title, xlabel, ylabel, fig_name=None):
                ax.plot(T1, Y1, "b", label=labels[0], lw=2)
                ax.plot([0, T1[-1]], [Y1[-1], Y1[-1]], 'k--', lw=1)
                ax.plot([0, T1[-1]], [1.05 * Y1[-1], 1.05 * Y1[-1]], 'k--', lw=1)
                ax.plot([0, T1[-1]], [0.95 * Y1[-1], 0.95 * Y1[-1]], 'k--', lw=1)
                ax.minorticks_on()
                ax.grid(True, which='both')
                ax.set_title(title)
                ax.set_xlabel(xlabel)
                ax.set_ylabel(ylabel)
                ax.legend()

                # Set window title and save figure if required
                if fig_name:
                    ax.figure.canvas.manager.set_window_title(fig_name)
                    if self.save_fig:
                        ax.figure.savefig(os.path.join(OUTPUT_FIG_PATH, f'{fig_name}.png'))
                else:
                    plt.show()
                    
        if label == 'q':
            # Calcul pour q
            self.matrix_A_new = self.matrix_A[1:, 1:]
            self.matrix_B_new = self.matrix_B[1:]
            self.matrix_C_q = np.zeros((5, 1))
            self.matrix_C_q[2] = 1  # 2 correspond à l'index de q
            self.matrix_C_q = self.matrix_C_q.T
            self.sys_q = control.ss(self.matrix_A_new, self.matrix_B_new, self.matrix_C_q, 0)
            self.TF_q = control.ss2tf(self.sys_q)
            self.gain_Kr = -0.20954 #sisopy31.sisotool(1 * minreal(self.TF_q))
            self.matrix_A_q = self.matrix_A_new - self.gain_Kr * np.dot(self.matrix_B_new, self.matrix_C_q.T)
            self.matrix_B_q = self.gain_Kr * self.matrix_B_new
            self.matrix_D_q = self.gain_Kr * self.matrix_D
            self.closedloop_sys_q = control.ss(self.matrix_A_q, self.matrix_B_q, self.matrix_C_q, 0)  # Create the state space system

            control.matlab.damp(self.closedloop_sys_q)
            self.Closed_Tf_ss_q = control.tf(self.closedloop_sys_q)

            fig, ax = plt.subplots()

            Yqcl, Tqcl = control.matlab.step(self.Closed_Tf_ss_q, np.arange(0, 5, 0.01))

            plot_step_response(
                ax=ax,
                T1=Tqcl,
                Y1=Yqcl,
                labels=[r"$q/q_c$"],
                title=r"Step response $q/q_c$",
                xlabel="Time (s)",
                ylabel=r"$q$ (rad/s)",
                fig_name="Step_Response_q_qc",
            )
            
            def plot_step_response_washout(t, y_alpha, y_alpha_no_washout, y_alpha_washout, title, xlabel, ylabel, fig_name=None):
                plt.figure()
                plt.plot(t, y_alpha, label="Alpha alpha", color="red")
                plt.plot(t, y_alpha_no_washout, label="Alpha alpha no washout", color="blue")
                plt.plot(t, y_alpha_washout, linestyle=(0, (5, 10)), color="green", label="Alpha alpha washout")
                plt.title(title)
                plt.grid()
                plt.legend()
                plt.xlabel(xlabel)
                plt.ylabel(ylabel)

                if fig_name:
                    if self.save_fig:
                        plt.savefig(os.path.join(OUTPUT_FIG_PATH, f"{fig_name}.png"))
                    else:
                        plt.show()
                else:
                    plt.show()
            
            tf_washout_filter = control.tf([self.tau, 0], [self.tau, 1])
            tf_washout_filter_closed = control.feedback(self.gain_Kr, self.TF_q * tf_washout_filter)

            C_alpha = [0, 1, 0, 0, 0]
            ss_alpha = control.ss(self.matrix_A_new, self.matrix_B_new, C_alpha, 0)

            tf_alpha = control.tf(ss_alpha)
            tf_alpha_washout = control.series(1 / self.gain_Kr, tf_washout_filter_closed, tf_alpha)
            tf_alpha_no_washout = control.series(1 / self.gain_Kr, control.feedback(self.gain_Kr, self.TF_q), tf_alpha)
            t = np.arange(0, 15, 0.01)

            y_alpha, t_alpha = control.matlab.step(tf_alpha, t)
            y_alpha_no_washout, t_alpha_no_washout = control.matlab.step(tf_alpha_no_washout, t)
            y_alpha_washout, t_alpha_washout = control.matlab.step(tf_alpha_washout, t)

            plot_step_response_washout(
                t=t_alpha,
                y_alpha=y_alpha,
                y_alpha_no_washout=y_alpha_no_washout,
                y_alpha_washout=y_alpha_washout,
                title="Washout filter",
                xlabel="Time (s)",
                ylabel=r"$\alpha$",
                fig_name="Washout_Filter_Alpha",
            )
                    

        elif label == 'gamma':
            # Calcul pour gamma
            self.matrix_C_gamma = np.zeros((5, 1))
            self.matrix_C_gamma[0] = 1  # 0 correspond à l'index de gamma
            self.matrix_C_gamma = self.matrix_C_gamma.T
            self.sys_gamma = control.ss(self.matrix_A_q, self.matrix_B_q, self.matrix_C_gamma, 0)
            self.TF_gamma = control.ss2tf(self.sys_gamma)
            self.gain_Kg = 16.4 #sisopy31.sisotool(1 * minreal(self.TF_gamma))
            self.matrix_A_gamma = self.matrix_A_q - self.gain_Kg * np.dot(self.matrix_B_q, self.matrix_C_gamma.T)
            self.matrix_B_gamma = self.gain_Kg * self.matrix_B_q
            self.matrix_D_gamma = self.gain_Kg * self.matrix_D

            self.closedloop_sys_gamma = control.ss(self.matrix_A_gamma, self.matrix_B_gamma, self.matrix_C_gamma, 0)
            control.matlab.damp(self.closedloop_sys_gamma)

            self.Closed_Tf_ss_gamma = control.tf(self.closedloop_sys_gamma)
            
            fig, ax = plt.subplots()
            Y_gamma_cl, T_gamma_cl = control.matlab.step(self.Closed_Tf_ss_gamma, np.arange(0, 5, 0.01))

            plot_step_response(
                ax=ax,
                T1=T_gamma_cl,
                Y1=Y_gamma_cl,
                labels=[r"$gamma/gamma_c$"],
                title=r"Step response $gamma/gamma_c$",
                xlabel="Time (s)",
                ylabel=r"$gamma$ (rad/s)",
                fig_name="Step_Response_gamma_gammac",
            )

        elif label == 'z':
            # Calcul pour z
            self.matrix_C_z = np.zeros((5, 1))
            self.matrix_C_z[4] = 1  # 4 correspond à l'index de z
            self.matrix_C_z = self.matrix_C_z.T
            self.sys_z = control.ss(self.matrix_A_gamma, self.matrix_B_gamma, self.matrix_C_z, 0)
            self.TF_z = control.ss2tf(self.sys_z)
            self.gain_Kz = 0.00010 #sisopy31.sisotool(1 * minreal(self.TF_z))
            self.matrix_A_z = self.matrix_A_gamma - self.gain_Kz * np.dot(self.matrix_B_gamma, self.matrix_C_z.T)
            self.matrix_B_z = self.gain_Kz * self.matrix_B_gamma
            self.matrix_D_z = self.gain_Kz * self.matrix_D

            self.closedloop_sys_z = control.ss(self.matrix_A_z, self.matrix_B_z, self.matrix_C_z, 0)
            control.matlab.damp(self.closedloop_sys_z)

            self.Closed_Tf_ss_z = control.tf(self.closedloop_sys_z)
            
            fig, ax = plt.subplots()
            Y_z_cl, T_z_cl = control.matlab.step(self.Closed_Tf_ss_z, np.arange(0, 5, 0.01))
            plot_step_response(
                ax=ax,
                T1=T_z_cl,
                Y1=Y_z_cl,
                labels=[r"$z/z_c$"],
                title=r"Step response $z/z_c$",
                xlabel="Time (s)",
                ylabel=r"$q$ (rad/s)",
                fig_name="Step_Response_z_zc",
            )
            
            #self.closed_state_space_z = control.ss(self.matrix_A_z, self.matrix_B_z, self.matrix_C_z, self.matrix_D_z)


    def ft_to_m(self, feet):
        """
        Convert a value in feet to meters.

        Parameters:
        - feet (float): Value in feet to be converted.

        Returns:
        - float: Equivalent value in meters.
        """
        meters = feet * 0.3048
        return meters
    
    



        