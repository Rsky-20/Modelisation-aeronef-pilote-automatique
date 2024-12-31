import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import TransferFunction, bode
import matplotlib.pyplot as plt
from control import ss, feedback, damp, tf, step_response
import math
import os
import sys
import traceback
import inspect


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

# Styles de texte
RESET = "\033[0m"        # Réinitialiser
BOLD = "\033[1m"         # Gras
DIM = "\033[2m"          # Atténué
ITALIC = "\033[3m"       # Italique (peut ne pas fonctionner dans toutes les consoles)
UNDERLINE = "\033[4m"    # Souligné
BLINK = "\033[5m"        # Clignotant (rarement supporté)
INVERT = "\033[7m"       # Inversé (texte et arrière-plan échangés)
STRIKETHROUGH = "\033[9m" # Barré

# Couleurs de texte (Foreground)
BLACK = "\033[30m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
GRAY = "\033[90m"
LIGHT_RED = "\033[91m"
LIGHT_GREEN = "\033[92m"
LIGHT_YELLOW = "\033[93m"
LIGHT_BLUE = "\033[94m"
LIGHT_MAGENTA = "\033[95m"
LIGHT_CYAN = "\033[96m"
BRIGHT_WHITE = "\033[97m"

# Couleurs de fond (Background)
BG_BLACK = "\033[40m"
BG_RED = "\033[41m"
BG_GREEN = "\033[42m"
BG_YELLOW = "\033[43m"
BG_BLUE = "\033[44m"
BG_MAGENTA = "\033[45m"
BG_CYAN = "\033[46m"
BG_WHITE = "\033[47m"
BG_GRAY = "\033[100m"
BG_LIGHT_RED = "\033[101m"
BG_LIGHT_GREEN = "\033[102m"
BG_LIGHT_YELLOW = "\033[103m"
BG_LIGHT_BLUE = "\033[104m"
BG_LIGHT_MAGENTA = "\033[105m"
BG_LIGHT_CYAN = "\033[106m"
BG_BRIGHT_WHITE = "\033[107m"


def display_time_process(func, *args, **kwargs):
    try:
        start_time = time.time()
        result = func(*args, **kwargs)  # Appelle la fonction/méthode avec les arguments
        end_time = time.time()

        execution_time = end_time - start_time
        func_name = func.__name__

        # Si la fonction est une lambda, tente de récupérer son code source
        if func_name == '<lambda>':
            try:
                func_code = inspect.getsource(func).strip()
                # Extrait le contenu après le `lambda:` dans le code source
                func_body = func_code.split(':', 1)[1].strip()
                func_name = f"{func_body[:-1]}"
            except Exception:
                func_name = "<lambda> (func.__name__ unavailable)"

        print(f"[{YELLOW}INFO{RESET}] - {BOLD}'{ITALIC}{func_name}{RESET}{BOLD}'{RESET} status : {GREEN}OK{RESET}")
        print(f"[{YELLOW}INFO{RESET}] - {BLUE}Processing time : {execution_time:.6f}s{RESET}")
        return result
    except ValueError as err:
        print(f"[{YELLOW}INFO{RESET}] - Process status : {RED}ERROR{RESET}")
        print(f"[{RED}ERROR{RESET}]\n{LIGHT_RED}{err}{RESET}")
        traceback.print_exc()  # Affiche la trace avec le message complet

class Display_CAM:
    def __init__(self, control_aircraft_model):
        self.CAM = control_aircraft_model
        
    def process(self, process_name):
        flag_line = f"#    {process_name}    #"
        print(f'{LIGHT_CYAN}',"#"*len(flag_line), f'{RESET}')
        print(f'{LIGHT_CYAN}', flag_line, f'{RESET}')
        print(f'{LIGHT_CYAN}',"#"*len(flag_line), f'{RESET}')
    
    def _plot(fig_name, save=False):
        pass
    
    def matrix(self):
        try:
            print(f"""
##############################################################
Matrice A :
{self.CAM.matrix_A}

Matrice B :
{self.CAM.matrix_B}

Matrice C :
{self.CAM.matrix_C}

Matrice D :
{self.CAM.matrix_D}
##############################################################
                    """)
        except:pass
        
    def equilibrium_condition_values(self):
        """ 
        Affiche les valeurs des conditions d'équilibre calculées.
        """
        print(f"""
##############################################################
EQUILIBRIUM CONDITIONS VALUES
--------------------------------------------------------------
Coefficient de portance à l'équilibre (C_z_eq):      {self.CAM.C_z_eq:.6f}
Coefficient de traînée à l'équilibre (C_x_eq):      {self.CAM.C_x_eq:.6f}
Coefficient de contrôle de traînée (C_x_delta_m):   {self.CAM.C_x_delta_m:.6f}
Déviation initiale de l'empennage (delta_m_eq):     {self.CAM.delta_m_eq:.6f}
Liste des angles d'équilibre (alpha_eq_list):       {self.CAM.alpha_eq_list}
Dernier angle d'équilibre (alpha_eq):              {self.CAM.alpha_eq:.6f}
Force de propulsion à l'équilibre (F_p_x_eq):       {self.CAM.F_p_x_eq:.6f}
Force aérodynamique totale à l'équilibre (F_eq):    {self.CAM.F_eq:.6f}
--------------------------------------------------------------
Coefficient de traînée dû à l'incidence (C_x_alpha): {self.CAM.C_x_alpha:.6f}
Coefficient de moment aérodynamique (C_m_alpha):    {self.CAM.C_m_alpha:.6f}
Coefficient de portance dû à l'incidence (Z_alpha): {self.CAM.Z_alpha:.6f}
Coefficient de portance dû à la gouverne (Z_delta_m):{self.CAM.Z_delta_m:.6f}
Moment dû à l'incidence (m_alpha):                 {self.CAM.m_alpha:.6f}
Moment de tangage (m_q):                           {self.CAM.m_q:.6f}
Moment dû à la gouverne (m_delta_m):               {self.CAM.m_delta_m:.6f}
Coefficient de moment dû à la gouverne (C_m_delta_m):{self.CAM.C_m_delta_m:.6f}
--------------------------------------------------------------
Coefficient de vitesse longitudinale (X_v):         {self.CAM.X_v:.6f}
Coefficient de tangage dû au cap (X_gamma):         {self.CAM.X_gamma:.6f}
Coefficient de tangage dû à l'incidence (X_alpha):  {self.CAM.X_alpha:.6f}
Coefficient de portance longitudinale (Z_v):        {self.CAM.Z_v:.6f}
##############################################################
        """)
        
    def state_space_info(self):
        print(f"""
##############################################################
STATE SPACE MODEL INFORMATION
{self.CAM.display_matrix()}

Fonction de transfert du système (numérateur/denominateur) :
{self.CAM.state_space_model_tf}
--------------------------------------------------------------
Analyse des pôles et caractéristiques :
--------------------------------------------------------------""")
        for i, pole in enumerate(self.CAM.pole_model[0]):
            print(f"Pôle {i+1}:")
            print(f"  - Valeur complexe : {pole}")
            print(f"  - Partie réelle : {self.CAM.pole_model[1][i]:.6f}")
            print(f"  - Partie imaginaire : {self.CAM.pole_model[2][i]:.6f}")
            print(f"  - Coefficient d'amortissement (ξ) : {self.CAM.pole_model[3][i]:.6f}")
            print(f"  - Fréquence naturelle (ω) : {self.CAM.pole_model[4][i]:.6f}")
            print("--------------------------------------------------------------")

        print(f"""
    Réglage du gain avec SISO Tool :
    --------------------------------------------------------------
    Gain sélectionné : {self.CAM.gain_k:.6f}
    ##############################################################
            """)


    def aircraft_parameters(self):
        """
        Affiche les paramètres de l'avion.
        """
        print(f"""
##############################################################
AIRCRAFT PARAMETERS
--------------------------------------------------------------
Nom de l'avion (name):                               {self.CAM.name}
--------------------------------------------------------------
CONSTANTS
Gravité (g):                                         {self.CAM.g:.2f} m/s^2
Constante de temps (tau):                            {self.CAM.tau:.2f} s
Rayon de giration (r_g):                             {self.CAM.r_g:.2f} m
--------------------------------------------------------------
FLIGHT CONDITION
Mach speed:                                          {self.CAM.mach_speed:.2f}
Altitude (altitude):                                 {self.CAM.altitude:.2f} m
Densité de l'air (rho):                              {self.CAM.rho:.6f} kg/m^3
Vitesse du son (V_s):                                {self.CAM.V_s:.2f} m/s
Vitesse aérodynamique (V_a):                         {self.CAM.V_a:.2f} m/s
--------------------------------------------------------------
AIRCRAFT CHARACTERISTICS
Masse (mass):                                        {self.CAM.mass:.2f} kg
Surface alaire de référence (S):                     {self.CAM.S:.2f} m^2
Longueur de référence (l_ref):                       {self.CAM.l_ref:.2f} m
Distance à l'empennage (lt):                         {self.CAM.lt:.2f} m
Centre de gravité (c):                               {self.CAM.c:.2f}
Moment d'inertie (I_yy):                             {self.CAM.I_yy:.2f} kg·m^2
--------------------------------------------------------------
AERODYNAMIC COEFFICIENTS
C_x_0 (Coefficient de traînée à angle nul):          {self.CAM.C_x_0:.6f}
C_z_alpha (Gradient de portance):                    {self.CAM.C_z_alpha:.6f}
C_z_delta_m (Gradient de portance de la gouverne):   {self.CAM.C_z_delta_m:.6f}
Déviation de l'empennage à l'équilibre (delta_m_0):  {self.CAM.delta_m_0:.6f}
Angle d'incidence à portance nulle (alpha_0):        {self.CAM.alpha_0:.6f} rad
Centre aérodynamique (f):                            {self.CAM.f:.2f}
Centre aérodynamique des gouvernes (f_delta):        {self.CAM.f_delta:.2f}
Coefficient polaire (k):                             {self.CAM.k:.6f}
Coefficient d'amortissement de tangage (C_m_q):      {self.CAM.C_m_q:.6f}
Coefficient de moment dû à la gouverne (C_m_delta):  {self.CAM.C_m_delta:.6f}
F_delta:                                             {self.CAM.F_delta:.2f}
F:                                                   {self.CAM.F:.2f}
Pression dynamique (Q):                              {self.CAM.Q:.2f} Pa
--------------------------------------------------------------
LEVER ARM DISTANCES
X (Distance entre F et G):                           {self.CAM.X:.2f} m
Y (Distance entre F_delta et G):                     {self.CAM.Y:.2f} m
--------------------------------------------------------------
ÉTAT D'ÉQUILIBRE
Vitesse d'équilibre (V_eq):                          {self.CAM.V_eq:.2f} m/s
Gamma à l'équilibre (gamma_eq):                      {self.CAM.gamma_eq:.6f}
##############################################################
    """)
        
    def open_loop(self):
        print(f"""
####################### EigenValues of A #######################
EigenValues :
{self.CAM.openloop_eigenA}

####################### Damping ratio and Pulsation #######################
                """)
        for eigen in self.CAM.openloop_eigen_list:
            print(f"Pôle : {eigen['eigen']}, wn : {eigen['wn']}, Xi : {eigen['Xi']}")

        
    def transient_phase(self):
        print(f"""
####################### OSCILLATION MODES #######################

    - Short period mode: 
        >> Alpha : 
        * Poles           :{self.CAM.sp_eigenvalues_alpha}
        * Proper pulsation: {self.CAM.sp_wn_alpha} rad/s
        * Damping ratio   : {self.CAM.sp_damping_alpha} 
        >> Q : 
        * Poles           :{self.CAM.sp_eigenvalues_q}
        * Proper pulsation: {self.CAM.sp_wn_q} rad/s
        * Damping ratio   : {self.CAM.sp_damping_q} 
    
    - Phugoid mode:
        >> Gamma :
        * Poles           :{self.CAM.phu_eigenvalues_g}
        * Proper pulsation: {self.CAM.phu_wn_g} rad/s
        * Damping ratio   : {self.CAM.phu_damping_g} 
        >> V :
        * Poles           :{self.CAM.phu_eigenvalues_v}
        * Proper pulsation: {self.CAM.phu_wn_v} rad/s
        * Damping ratio   : {self.CAM.phu_damping_v} 
              
#################### STATE SPACE REPRESENTATION ##################

    - Short period mode: 
        >> Alpha    : \n{self.CAM.ss_sp_alpha}
        >> Q        : \n{self.CAM.ss_sp_q}
    
    - Phugoid mode:
        >> Gamma    : \n{self.CAM.ss_phu_g}
        >> V        : \n{self.CAM.ss_phu_v}

####################### TRANSFER FUNCTIONS #######################

    - Short period mode:
            - Alpha: \n{str(self.CAM.tf_sp_alpha)}
            - q    : \n{str(self.CAM.tf_sp_q)}
            
    - Phugoid mode:
            - V     : \n{str(self.CAM.tf_phu_v)}
            - Gamma : \n{str(self.CAM.tf_phu_g)}

########################## SETTLING TIME #########################

    - Short period mode:
        - Alpha Settling time   : {self.CAM.Tsa}
        - Q Settling time       : {self.CAM.Tsq}
        
    - Phugoid mode:
        - V Settling time       : {self.CAM.Tsv}
        - Gamma Settling time   : {self.CAM.Tsg}
        
########################## GAIN with SISOPY31 #########################

    - Short period mode:
            - Alpha gain   : {self.CAM.gain_k_sp_alpha}
            - Q gain       : {self.CAM.gain_k_sp_q}

    - Phugoid mode:
            - V gain       : {self.CAM.gain_k_phu_v}
            - Gamma gain   : {self.CAM.gain_k_phu_g}
        
              """)

    def closedloop_TF(self, label:str):
        print(f"""
########################## Transfer Function of the {label} closed loop #########################
                  """)
        if label == 'q':
            print(self.CAM.Closed_Tf_ss_q)
        if label == 'gamma':
            print(self.CAM.Closed_Tf_ss_gamma)
        if label == 'z':
            print(self.CAM.Closed_Tf_ss_z)

    def gain_k(self):
        print(f"""
########################## GAIN with SISOPY31 #########################

>> Gain K for Q feedback loop : {self.CAM.gain_Kr}
>> Gain K for gamma feedback loop : {self.CAM.gain_Kg}
>> Gain K for z feedback loop : {self.CAM.gain_Kz}
""")
