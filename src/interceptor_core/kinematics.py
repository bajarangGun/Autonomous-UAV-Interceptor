"""
Kinematics Module for 3D Interception.

Calculates relative kinematics (Closing Velocity and Line-of-Sight Rate)
using absolute 3D position and velocity truth data from a simulator.
"""

import math
import logging

logger = logging.getLogger(__name__)

def calculate_los_kinematics(
    pos_interceptor: tuple[float, float, float],
    pos_target: tuple[float, float, float],
    vel_interceptor: tuple[float, float, float],
    vel_target: tuple[float, float, float]
) -> tuple[float, tuple[float, float, float]]:
    """
    Calculates Closing Velocity and the 3D Line-of-Sight (LOS) Rate Vector.
    
    Args:
        pos_interceptor: (X, Y, Z) position of the drone in meters.
        pos_target: (X, Y, Z) position of the target in meters.
        vel_interceptor: (Vx, Vy, Vz) velocity of the drone in m/s.
        vel_target: (Vx, Vy, Vz) velocity of the target in m/s.
        
    Returns:
        tuple containing:
            - closing_velocity (float): Speed at which the distance is decreasing (m/s).
            - los_rate_vector (tuple): (Wx, Wy, Wz) rotational rate of the LOS vector (rad/s).
    """
    
    # 1. Relative Position Vector (R = P_target - P_interceptor)
    rx = pos_target[0] - pos_interceptor[0]
    ry = pos_target[1] - pos_interceptor[1]
    rz = pos_target[2] - pos_interceptor[2]
    
    # 2. Relative Velocity Vector (V_rel = V_target - V_interceptor)
    vx = vel_target[0] - vel_interceptor[0]
    vy = vel_target[1] - vel_interceptor[1]
    vz = vel_target[2] - vel_interceptor[2]
    
    # Calculate range squared
    range_sq = rx**2 + ry**2 + rz**2
    
    # Defensive check: Avoid division by zero if we have actually hit the target
    if range_sq < 0.001:
        logger.warning("Target intercept achieved (range ~ 0).")
        return 0.0, (0.0, 0.0, 0.0)
        
    r_mag = math.sqrt(range_sq)
    
    # 3. Closing Velocity (Vc) 
    # Vc = - (R dot V_rel) / |R| 
    # (Positive means the distance is shrinking)
    r_dot_v = (rx * vx) + (ry * vy) + (rz * vz)
    closing_velocity = - (r_dot_v / r_mag)
    
    # 4. Line-of-Sight Rate Vector (Omega)
    # Omega = (R cross V_rel) / |R|^2
    cross_x = (ry * vz) - (rz * vy)
    cross_y = (rz * vx) - (rx * vz)
    cross_z = (rx * vy) - (ry * vx)
    
    los_rate_x = cross_x / range_sq
    los_rate_y = cross_y / range_sq
    los_rate_z = cross_z / range_sq
    
    return closing_velocity, (los_rate_x, los_rate_y, los_rate_z)
