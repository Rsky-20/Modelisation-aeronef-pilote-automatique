import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import TransferFunction, bode
import matplotlib.pyplot as plt
from control import ss, feedback, damp, tf, step_response
import math
import os
import sys

try:
    from sisopy31 import *
except:
    from lib.sisopy31 import *

try:
    import atm_std
except:
    import lib.atm_std as atm_std
    
import time

class ControlAircraftModel:
    def __init__(self, name='MIRAGE III class'):
        self.name = name
        # Constants
        self.g = 9.81     # Gravitational acceleration (m/s^2)
        self.tau = 0.75   # time constant for transfert function (s)
        self.r_g = 2.65 #m

        # Flight condition 
        self.mach_speed = 1.88  # Aircraft Mach speed (relative to speed of sound)
        self.altitude = self.ft_to_m(22265)   # Aircraft cruise altitude (ft)
        self.hgeo, self.rho, self.V_s =  atm_std.get_cte_atm(self.altitude) #hgeo = , roh = sensité de l'air, V_s = vitesse du son 
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
        self.C_x_0 = 0.0259         # Drag coefficient at zero angle of attack
        self.C_z_alpha = 2.2    # Lift gradient coefficient w.r.t. angle of attack
        self.C_z_delta_m = 0.32     # Lift gradient coefficient w.r.t. elevator deflection
        self.delta_m_0 = -0.01   # Equilibrium fin deflection for null lift 
        self.alpha_0 = 0.011    # Incidence for null lift and null fin deflection (rad)
        self.f = 0.68          # Aerodynamique center of body and wings
        self.f_delta = 0.9      # Aerodynamique center of fins (pitch axis)
        self.k = 0.515           # Polar coefficient 
        self.C_m_q = -0.255        # Pitch damping coefficient (s/rad)
        self.C_m_delta = -1.1    # Moment coefficient w.r.t. elevator deflection
        self.F_delta = self.f_delta * self.lt
        self.F = self.f * self.lt
        
        self.Q = 0.5 * self.rho * pow(self.V_a, 2)

        #X,Y bras de levier distance entre F et G :  X_g - X_f
        self.X =  -(self.f * self.lt - self.c * self.lt)
        self.Y =  -(self.F_delta - self.c * self.lt)

        self.V_eq = self.V_a #car on est pas loin du point d'équilibre 
        
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
        
        self.state_space_model = None
        self.state_space_model_tf = None
        self.gain_k = None
        self.pole_model = None

    
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
        return Y/l_ref * (C_x_delta_m * np.sin(alpha_eq) + C_z_delta_m * np.cos(alpha_eq))
    
    def compute_C_x_alpha(self, k, C_z, C_z_alpha):
        return 2 * k * C_z * C_z_alpha
    
    def compute_C_m_alpha(self, X, l_ref, C_x_alpha, alpha_eq, C_z_alpha):
        return (X / l_ref) * (C_x_alpha * np.sin(alpha_eq)+C_z_alpha * np.cos(alpha_eq))
    
    
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
                                    (C_x_delta_m * np.sin(alpha_eq) + self.C_z_delta_m * np.cos(alpha_eq))) * self.X / (self.Y - self.X)
            F_p_x_eq = self.Q * self.S * C_x_eq / np.cos(alpha_eq)
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
        self.X_alpha = self.compute_X_gamma(self.g, self.gamma_eq, self.V_eq)
        
    
    def K1(self, V_eq, m_alpha, Z_delta_m, m_delta_m, Z_alpha, m_q):
        return (-V_eq * (-m_alpha * Z_delta_m + m_delta_m * Z_alpha)) / (-m_q * Z_alpha - m_alpha)
    
    def omega(self, Z_delta_m, m_alpha, Z_alpha, m_delta_m):
        return math.sqrt(-Z_delta_m * (-m_alpha * Z_delta_m + m_delta_m * Z_alpha)) / Z_delta_m
    
    def tau_z(self, Z_delta_m, m_q, m_alpha, m_delta_m, Z_alpha):
        return -((Z_delta_m * m_q) / (-m_alpha * Z_delta_m + m_delta_m * Z_alpha))
    
    def open_loop(self):
        
        # Création du modèle état-espace complet
        self.state_space_model = ss(self.matrix_A, self.matrix_B, self.matrix_C[1], 0)

        # Conversion en fonction de transfert
        self.state_space_model_tf = tf(self.state_space_model)

        # Affichage des informations sur les pôles
        print("\nAnalyse des pôles pour le modèle complet :")
        self.pole_model = damp(self.state_space_model)

        # Utilisation de sisotool pour ajuster le gain (facultatif)
        print("\nOuverture de sisotool pour ajuster le gain :")
        self.gain_k = sisotool(minreal(self.state_space_model)) #, kmin=0.001, kmax=10.0, kdefault=1.0, xispec=0.7


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
    
    def display_matrix(self):
        try:
            print(f"""
##############################################################
Matrice A :
{self.matrix_A}

Matrice B :
{self.matrix_B}

Matrice C :
{self.matrix_C}

Matrice D :
{self.matrix_D}
##############################################################
                    """)
        except:pass
        
    def display_equilibrium_condition_values(self):
        """
        Affiche les valeurs des conditions d'équilibre calculées.
        """
        print(f"""
##############################################################
EQUILIBRIUM CONDITIONS VALUES
--------------------------------------------------------------
Coefficient de portance à l'équilibre (C_z_eq):      {self.C_z_eq:.6f}
Coefficient de traînée à l'équilibre (C_x_eq):      {self.C_x_eq:.6f}
Coefficient de contrôle de traînée (C_x_delta_m):   {self.C_x_delta_m:.6f}
Déviation initiale de l'empennage (delta_m_eq):     {self.delta_m_eq:.6f}
Liste des angles d'équilibre (alpha_eq_list):       {self.alpha_eq_list}
Dernier angle d'équilibre (alpha_eq):              {self.alpha_eq:.6f}
Force de propulsion à l'équilibre (F_p_x_eq):       {self.F_p_x_eq:.6f}
Force aérodynamique totale à l'équilibre (F_eq):    {self.F_eq:.6f}
--------------------------------------------------------------
Coefficient de traînée dû à l'incidence (C_x_alpha): {self.C_x_alpha:.6f}
Coefficient de moment aérodynamique (C_m_alpha):    {self.C_m_alpha:.6f}
Coefficient de portance dû à l'incidence (Z_alpha): {self.Z_alpha:.6f}
Coefficient de portance dû à la gouverne (Z_delta_m):{self.Z_delta_m:.6f}
Moment dû à l'incidence (m_alpha):                 {self.m_alpha:.6f}
Moment de tangage (m_q):                           {self.m_q:.6f}
Moment dû à la gouverne (m_delta_m):               {self.m_delta_m:.6f}
Coefficient de moment dû à la gouverne (C_m_delta_m):{self.C_m_delta_m:.6f}
--------------------------------------------------------------
Coefficient de vitesse longitudinale (X_v):         {self.X_v:.6f}
Coefficient de tangage dû au cap (X_gamma):         {self.X_gamma:.6f}
Coefficient de tangage dû à l'incidence (X_alpha):  {self.X_alpha:.6f}
Coefficient de portance longitudinale (Z_v):        {self.Z_v:.6f}
##############################################################
        """)
        
    def display_state_space_info(self):
        print(f"""
##############################################################
STATE SPACE MODEL INFORMATION
{self.display_matrix()}

Fonction de transfert du système (numérateur/denominateur) :
{self.state_space_model_tf}
--------------------------------------------------------------
Analyse des pôles et caractéristiques :
--------------------------------------------------------------""")
        for i, pole in enumerate(self.pole_model[0]):
            print(f"Pôle {i+1}:")
            print(f"  - Valeur complexe : {pole}")
            print(f"  - Partie réelle : {self.pole_model[1][i]:.6f}")
            print(f"  - Partie imaginaire : {self.pole_model[2][i]:.6f}")
            print(f"  - Coefficient d'amortissement (ξ) : {self.pole_model[3][i]:.6f}")
            print(f"  - Fréquence naturelle (ω) : {self.pole_model[4][i]:.6f}")
            print("--------------------------------------------------------------")

        print(f"""
    Réglage du gain avec SISO Tool :
    --------------------------------------------------------------
    Gain sélectionné : {self.gain_k:.6f}
    ##############################################################
            """)


    def display_aircraft_parameters(self):
        """
        Affiche les paramètres de l'avion.
        """
        print(f"""
##############################################################
AIRCRAFT PARAMETERS
--------------------------------------------------------------
Nom de l'avion (name):                                {self.name}
--------------------------------------------------------------
CONSTANTS
Gravité (g):                                         {self.g:.2f} m/s^2
Constante de temps (tau):                            {self.tau:.2f} s
Rayon de giration (r_g):                             {self.r_g:.2f} m
--------------------------------------------------------------
FLIGHT CONDITION
Mach speed:                                          {self.mach_speed:.2f}
Altitude (altitude):                                 {self.altitude:.2f} m
Densité de l'air (rho):                              {self.rho:.6f} kg/m^3
Vitesse du son (V_s):                                {self.V_s:.2f} m/s
Vitesse aérodynamique (V_a):                         {self.V_a:.2f} m/s
--------------------------------------------------------------
AIRCRAFT CHARACTERISTICS
Masse (mass):                                        {self.mass:.2f} kg
Surface alaire de référence (S):                     {self.S:.2f} m^2
Longueur de référence (l_ref):                       {self.l_ref:.2f} m
Distance à l'empennage (lt):                         {self.lt:.2f} m
Centre de gravité (c):                               {self.c:.2f}
Moment d'inertie (I_yy):                             {self.I_yy:.2f} kg·m^2
--------------------------------------------------------------
AERODYNAMIC COEFFICIENTS
C_x_0 (Coefficient de traînée à angle nul):          {self.C_x_0:.6f}
C_z_alpha (Gradient de portance):                    {self.C_z_alpha:.6f}
C_z_delta_m (Gradient de portance de la gouverne):   {self.C_z_delta_m:.6f}
Déviation de l'empennage à l'équilibre (delta_m_0):  {self.delta_m_0:.6f}
Angle d'incidence à portance nulle (alpha_0):        {self.alpha_0:.6f} rad
Centre aérodynamique (f):                            {self.f:.2f}
Centre aérodynamique des gouvernes (f_delta):        {self.f_delta:.2f}
Coefficient polaire (k):                             {self.k:.6f}
Coefficient d'amortissement de tangage (C_m_q):      {self.C_m_q:.6f}
Coefficient de moment dû à la gouverne (C_m_delta):  {self.C_m_delta:.6f}
F_delta:                                             {self.F_delta:.2f}
F:                                                   {self.F:.2f}
Pression dynamique (Q):                              {self.Q:.2f} Pa
--------------------------------------------------------------
LEVER ARM DISTANCES
X (Distance entre F et G):                           {self.X:.2f} m
Y (Distance entre F_delta et G):                     {self.Y:.2f} m
--------------------------------------------------------------
ÉTAT D'ÉQUILIBRE
Vitesse d'équilibre (V_eq):                          {self.V_eq:.2f} m/s
Gamma à l'équilibre (gamma_eq):                      {self.gamma_eq:.6f}
##############################################################
    """)


    
    @staticmethod
    def display_time_process(func, *args, **kwargs):
        """
        Mesure le temps d'exécution d'une fonction ou méthode.
        
        :param func: La fonction ou méthode à mesurer.
        :param args: Les arguments positionnels pour la fonction/méthode.
        :param kwargs: Les arguments nommés pour la fonction/méthode.
        :return: Le résultat de l'exécution de la fonction/méthode.
        """
        start_time = time.time()
        result = func(*args, **kwargs)  # Appelle la fonction/méthode avec les arguments
        end_time = time.time()

        execution_time = end_time - start_time
        print(f"INFO - [Temps d'exécution de '{func.__name__}': {execution_time:.6f} secondes]")
        return result

        

# Example usage
if __name__ == "__main__":
    pass
