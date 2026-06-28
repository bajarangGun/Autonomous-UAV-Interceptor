"""
Unit tests for the 3D Kinematics Math Module.
"""

import pytest
from interceptor_core.kinematics import calculate_los_kinematics

def test_head_on_collision() -> None:
    """
    Test a head-on collision. LOS rate should be 0, 
    Closing velocity should be the sum of speeds.
    """
    pos_drone = (0.0, 0.0, 0.0)
    vel_drone = (10.0, 0.0, 0.0)  # Flying straight along X at 10 m/s
    
    pos_target = (100.0, 0.0, 0.0)
    vel_target = (-10.0, 0.0, 0.0) # Flying straight backwards along X at 10m/s
    
    vc, los_rate = calculate_los_kinematics(pos_drone, pos_target, vel_drone, vel_target)
    
    assert vc == 20.0  # 10 m/s + 10 m/s head-on
    assert los_rate == (0.0, 0.0, 0.0)  # No lateral drift

def test_crossing_target() -> None:
    """
    Test a target crossing perfectly horizontally in front of a hovering drone.
    """
    pos_drone = (0.0, 0.0, 0.0)
    vel_drone = (0.0, 0.0, 0.0)  # Hovering
    
    # Target is 100m straight ahead, moving left along the Y axis at 10m/s
    pos_target = (100.0, 0.0, 0.0)
    vel_target = (0.0, 10.0, 0.0)
    
    vc, los_rate = calculate_los_kinematics(pos_drone, pos_target, vel_drone, vel_target)
    
    # Distance isn't closing initially, it's a perfect cross
    assert vc == 0.0 
    
    # LOS Rate Z (Yaw) should be positive. 
    # Math: (Rx*Vy - Ry*Vx) / R^2 = (100*10 - 0*0) / 10000 = 1000/10000 = 0.1 rad/s
    assert los_rate[0] == 0.0
    assert los_rate[1] == 0.0
    assert los_rate[2] == 0.1

def test_zero_range() -> None:
    """
    Ensure the math handles dividing by zero when interception is achieved.
    """
    pos_drone = (10.0, 10.0, 10.0)
    pos_target = (10.0, 10.0, 10.0) # Exact same position
    vel_drone = (5.0, 0.0, 0.0)
    vel_target = (5.0, 0.0, 0.0)
    
    vc, los_rate = calculate_los_kinematics(pos_drone, pos_target, vel_drone, vel_target)
    
    assert vc == 0.0
    assert los_rate == (0.0, 0.0, 0.0)
