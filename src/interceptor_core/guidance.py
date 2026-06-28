"""
Proportional Navigation (PN) Guidance Module.

This module calculates intercept trajectories based on Line-of-Sight (LOS) rates.
Designed for a Quadrotor Tailsitter in forward flight mode.
"""

import logging

# Configure basic logging for the module
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

def calculate_lateral_acceleration(
    closing_velocity_ms: float, 
    los_rate_rads: float, 
    nav_constant: float = 3.0
) -> float:
    """
    Calculates the lateral acceleration command using Proportional Navigation.
    
    For a tailsitter pointing its nose at the target, this lateral acceleration 
    is achieved by pitching or yawing the drone while maintaining forward thrust.

    Args:
        closing_velocity_ms (float): The relative speed at which the drone is 
                                     approaching the balloon (meters per second).
        los_rate_rads (float): The rate at which the balloon is drifting across 
                               the camera's field of view (radians per second).
        nav_constant (float): The Navigation Constant (N). Industry standard is 
                              between 3.0 and 5.0. Defaults to 3.0.

    Returns:
        float: Commanded lateral acceleration (meters per second squared).
        
    Raises:
        ValueError: If closing velocity is negative (target is escaping).
    """
    
    # Defensive Programming: Check for impossible or dangerous states
    if closing_velocity_ms < 0.0:
        logger.error("Negative closing velocity detected. Target is moving away faster than drone.")
        raise ValueError("Closing velocity cannot be negative during terminal intercept.")
        
    if nav_constant <= 0.0:
        logger.error(f"Invalid navigation constant: {nav_constant}. Must be > 0.")
        raise ValueError("Navigation constant must be strictly positive.")

    # Calculate Proportional Navigation Command
    # Formula: a = N * Vc * d(lambda)/dt
    acceleration_cmd = nav_constant * closing_velocity_ms * los_rate_rads
    
    logger.debug(
        f"Calculated Accel: {acceleration_cmd:.2f} m/s^2 | "
        f"Vc: {closing_velocity_ms} m/s | LOS Rate: {los_rate_rads} rad/s"
    )
    
    return acceleration_cmd