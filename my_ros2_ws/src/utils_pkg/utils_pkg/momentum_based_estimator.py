import numpy as np
from scipy.spatial.transform import Rotation

class MomentumBasedEstimator:
    def __init__(self, mass, ix, iy, iz, ts, g0=9.81):
        self.ts = ts
        self.mass = mass
        self.J = np.diag([ix, iy, iz])
        self.g0 = g0
        
        # -- Taratura TRASLAZIONALE (forze esterne F_ext) ---------------------
        # Ta_T: tempo di assestamento al 5% [s]. 
        # zita_T: smorzamento (1.0 = critico).
        Ta_T   = 0.5   # [s] 
        zita_T = 0.9
        omega_n_T  = 3.0 / (Ta_T * zita_T)
        self.K1_T  = 2.0 * zita_T * omega_n_T
        self.K2_T  = omega_n_T / (2.0 * zita_T)

        # -- Taratura ROTAZIONALE (coppie esterne Tau_ext) --------------------
        Ta_R   = 0.4   # [s] 
        zita_R = 1.1
        omega_n_R  = 3.0 / (Ta_R * zita_R)
        self.K1_R  = 2.0 * zita_R * omega_n_R
        self.K2_R  = omega_n_R / (2.0 * zita_R)

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
        
        # Quantità di moto attuali (dai sensori) (generalized momentum, q nel paper)
        p_T = self.mass * np.array(v_k)
        p_R = self.J @ np.array(w_k)

        # Termini Noti (Wrench Nominale)
        # Nota: quat_k arriva dall'MPC come [w, x, y, z]. SciPy vuole [x, y, z, w].
        q_scipy = [quat_k[1], quat_k[2], quat_k[3], quat_k[0]]
        Rb = Rotation.from_quat(q_scipy).as_matrix()

        # Calcolo wrench nominale (matricione dell'integrale interno nel paper)
        # Forza nominale (mondo ENU) = Spinta ruotata - Gravità
        F_nom = Rb @ np.array([0.0, 0.0, Fz_prev]) - np.array([0.0, 0.0, self.mass * self.g0])
        
        # Coppia nominale (body FLU) = Coppia netta - Effetto di Coriolis
        tau_nom = np.array(tau_prev) - np.cross(w_k, self.J @ w_k)

        # ── Aggiornamento canale TRASLAZIONALE ───────────────────────────────
        dI_T = F_nom + self.r_T
        dr_T = self.K1_T * (-self.r_T + self.K2_T * (p_T - self.I_T))
        self.I_T += dI_T * self.ts
        self.r_T += dr_T * self.ts

        # ── Aggiornamento canale ROTAZIONALE ─────────────────────────────────
        dI_R = tau_nom + self.r_R
        dr_R = self.K1_R * (-self.r_R + self.K2_R * (p_R - self.I_R))
        self.I_R += dI_R * self.ts
        self.r_R += dr_R * self.ts

        return self.r_T.copy(), self.r_R.copy()