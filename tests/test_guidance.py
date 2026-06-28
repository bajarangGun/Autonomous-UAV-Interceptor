"""
Unit tests for the Proportional Navigation Guidance Module.
"""

import pytest

# Notice we drop "src." because pyproject.toml tells pytest that src/ is the root
from interceptor_core.guidance import calculate_lateral_acceleration

def test_calculate_lateral_acceleration_positive_rate() -> None:
    """Test standard interception with a target drifting right."""
    accel = calculate_lateral_acceleration(
        closing_velocity_ms=15.0, 
        los_rate_rads=0.1, 
        nav_constant=3.0
    )
    assert accel == 4.5

def test_calculate_lateral_acceleration_negative_rate() -> None:
    """Test standard interception with a target drifting left."""
    accel = calculate_lateral_acceleration(
        closing_velocity_ms=20.0, 
        los_rate_rads=-0.05, 
        nav_constant=4.0
    )
    assert accel == -4.0

def test_calculate_lateral_acceleration_zero_rate() -> None:
    """Test collision course (no drift). Acceleration should be zero."""
    accel = calculate_lateral_acceleration(
        closing_velocity_ms=10.0, 
        los_rate_rads=0.0, 
        nav_constant=3.0
    )
    assert accel == 0.0

def test_calculate_lateral_acceleration_negative_closing_velocity() -> None:
    """Ensure the system safely catches the error if the target is outrunning the drone."""
    with pytest.raises(ValueError, match="Closing velocity cannot be negative"):
        calculate_lateral_acceleration(
            closing_velocity_ms=-5.0, 
            los_rate_rads=0.1
        )


