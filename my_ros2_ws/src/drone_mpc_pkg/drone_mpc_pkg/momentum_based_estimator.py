import numpy as np
from scipy.spatial.transform import Rotation

class MomentumBasedEstimator:
    def __init__(self, mass, ix, iy, iz, ts, g0=9.81):
        self.ts = ts
        self.mass = mass
        self.J = np.diag([ix, iy, iz])
        self.g0 = g0
        
        # Tuning parameters (Secondo Ordine)
        self.Ta = 1     # Tempo di assestamento al 5% [s]
        self.zita = 0.9   # Smorzamento
        self.omega_n = 3.0 / (self.Ta * self.zita)

        # Calcolo dei guadagni K1 e K2 (scalari, validi per tutti gli assi)
        self.K1 = 2.0 * self.zita * self.omega_n
        self.K2 = self.omega_n / (2.0 * self.zita)

        # Variabili di stato dell'estimatore
        self.I_T = None  # Integrale traslazionale (Quantità di moto lineare attesa)
        self.I_R = None  # Integrale rotazionale (Quantità di moto angolare attesa)
        self.r_T = None  # Residuo traslazionale (Forza stimata)
        self.r_R = None  # Residuo rotazionale (Coppia stimata)

    def initialize(self, v0, w0):
        """ Inizializza gli accumulatori (integrali) con la quantità di moto iniziale reale """
        self.I_T = self.mass * np.array(v0, dtype=float)
        self.I_R = self.J @ np.array(w0, dtype=float)
        
        self.r_T = np.zeros(3)
        self.r_R = np.zeros(3)

    def update(self, v_k, w_k, quat_k, Fz_prev, tau_prev):
        """ Esegue il passo di integrazione di Eulero in avanti a (1/ts) Hz """
        
        # 1. Quantità di moto attuali (dai sensori)
        p_T = self.mass * np.array(v_k)
        p_R = self.J @ np.array(w_k)

        # 2. Termini Noti (Wrench Nominale)
        # Nota: quat_k arriva dall'MPC come [w, x, y, z]. SciPy vuole [x, y, z, w].
        q_scipy = [quat_k[1], quat_k[2], quat_k[3], quat_k[0]]
        Rb = Rotation.from_quat(q_scipy).as_matrix()

        # Forza nominale (mondo) = Spinta ruotata - Gravità
        F_nom = Rb @ np.array([0.0, 0.0, Fz_prev]) - np.array([0.0, 0.0, self.mass * self.g0])
        
        # Coppia nominale (body) = Coppia netta - Effetto di Coriolis
        tau_nom = np.array(tau_prev) - np.cross(w_k, self.J @ w_k)

        # 3. Aggiornamento Derivate (Sistema dinamico)
        dI_T = F_nom + self.r_T
        dr_T = self.K1 * (-self.r_T + self.K2 * (p_T - self.I_T))

        dI_R = tau_nom + self.r_R
        dr_R = self.K1 * (-self.r_R + self.K2 * (p_R - self.I_R))

        # 4. Integrazione nel tempo (Metodo di Eulero in avanti)
        self.I_T += dI_T * self.ts
        self.r_T += dr_T * self.ts

        self.I_R += dI_R * self.ts
        self.r_R += dr_R * self.ts

        return self.r_T.copy(), self.r_R.copy()