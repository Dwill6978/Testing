#Script to run analysis on Gliders Configurations
#Author: Dylan Williams

import aerosandbox as asb
import array as arr
import aerosandbox.numpy as np


def StallSpeed(W_S, C_L_max, rho):
    # Calculate the stall speed of a glider

# Equation from Mechanics of Flight: Eq. 3.8.3
# Parameters: W_S = Wing Loading(N/m^2), C_L_max = Maximum Lift Coefficient, rho = Air Density (kg/m^3)

    ss = ((2* W_S) / (rho * C_L_max)) ** 0.5
    return ss

def GlideRatioSpeed(W_S, C_D_0, k, rho):
    # Calculate the Velocity for maximum glide ratio

# Equation from Aircraft Design: A Conceptual Approach Eq. 17.65
# Parameters: W_S = Wing Loading(N/m^2), C_D_0 = Zero-Lift Drag, K = Drag due to Lift Factor, rho = Air Density (kg/m^3)

    v = (W_S * (2/rho) * ((k/C_D_0)**0.5))**0.5
    return v

def MinSinkRateSpeed(W_S, C_D_0, k, rho):
    # Calculate the Velocity for minimum sink rate

# Equation from Aircraft Design: A Conceptual Approach Eq. 17.73
# Parameters: W_S = Wing Loading(N/m^2), C_D_0 = Zero-Lift Drag, K = Drag due to Lift Factor, rho = Air Density (kg/m^3)

    v = (W_S* (2/rho) * ((k/(3*C_D_0))**0.5))** 0.5
    return v

def TurnRadius(W_S, C_L, g, phi, rho):
    # Calculate the turn radius of a glider

# Equation from Aircraft Design: A Conceptual Approach Eq. 17.81
# Parameters: W_S = Wing Loading(N/m^2), C_L = Lift Coefficient, g = Acceleration due to Gravity (m/s^2), phi = Bank Angle (degrees), rho = Air Density (kg/m^3)

    r = (2/rho)*(W_S)/(C_L*g*np.sin(np.radians(phi)))
    return r

if __name__ == '__main__':

    # Example usage
    W_S = 1.422/0.1087  # N/m^2
    C_L_max = arr.array('f',[0.9,0.55,0.48])  # Maximum Lift Coefficient
    rho = 1.225  # kg/m^3 (sea level standard atmosphere)
    C_D_0 = arr.array('f',[0.019,0.035,0.05]) # Zero-Lift Drag Coefficient
    AR = 9.09 # Aspect Ratio
    e = 0.95 # Oswald Efficiency Factor
    k = 1/(AR * np.pi * e)  # Drag due to Lift Factor
    g = 9.81  # m/s^2
    phi = 30  # degrees

    # Plate C = 0.06 T = 0.05 R_LE = 2.11 Configuration
    print("Glider Analysis Results w/ Cambered Plate:")

    ss = StallSpeed(W_S, C_L_max[0], rho)
    print(f"Stall Speed: {ss:.2f} m/s")

    v_gr = GlideRatioSpeed(W_S, C_D_0[0], k, rho)
    print(f"Velocity for Maximum Glide Ratio: {v_gr:.2f} m/s")

    v_msr = MinSinkRateSpeed(W_S, C_D_0[0], k, rho)
    print(f"Velocity for Minimum Sink Rate: {v_msr:.2f} m/s")

    r = TurnRadius(W_S, C_L_max[0], g, phi, rho)
    print(f"Turn Radius: {r:.2f} m")




