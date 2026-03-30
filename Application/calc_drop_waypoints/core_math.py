from dataclasses import dataclass, field
from typing import Callable

import numpy as np


@dataclass
class SimulationEnvironment:
    # env variables, wind should be passed as a function
    air_density: float = 1.225  # use air_density.py
    gravity_vector: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -9.81])
    )  # use online calculator for specific location
    target_elevation: float = 0.0
    wind_model: Callable[[np.ndarray, float], np.ndarray] = field(
        default_factory=lambda: create_constant_wind(np.array([0.0, 0.0, 0.0]))
    )


def create_constant_wind(
    wind_vector: np.ndarray,
) -> Callable[[np.ndarray, float], np.ndarray]:
    """Constant wind model for env"""

    def constant_wind(position: np.ndarray, time: float) -> np.ndarray:
        return wind_vector

    return constant_wind


def create_shear_wind(
    base_wind_vector: np.ndarray, shear_exponent: float
) -> Callable[[np.ndarray, float], np.ndarray]:
    """Wind model where wind speed increases with altitude following a power law."""

    def shear_wind(position: np.ndarray, time: float) -> np.ndarray:
        height = max(position[2], 0.0)
        return base_wind_vector * ((height / 10.0) ** shear_exponent)

    return shear_wind


def create_logarithmic_wind(
    wind_vector_ref: np.ndarray, z_ref: float = 60.0, z0: float = 0.03
) -> Callable[[np.ndarray, float], np.ndarray]:
    """Logarithmic wind profile

    Args:
    wind_vector_ref: Wind vector at reference height (e.g., 60m)
    z_ref: Reference height for the given wind vector
    z0: Surface roughness length (this depends on terrain)
    """

    def log_wind(position: np.ndarray, time: float) -> np.ndarray:
        z = position[2]
        if z <= z0:
            return np.zeros_like(wind_vector_ref)
        scale = np.log(z / z0) / np.log(z_ref / z0)
        return wind_vector_ref * scale

    return log_wind


def create_drag_area_model(
    cd_payload: float,
    area_payload: float,
    cd_parachute: float,
    area_parachute: float,
    deploy_time: float,
    opening_duration: float,
) -> Callable[[float], float]:
    """Returns a time-dependent function for total Drag Area (Cd * A), here we need to adjust parameters to get the simulation right"""
    s_payload = cd_payload * area_payload
    s_chute = cd_parachute * area_parachute

    def drag_area(t: float) -> float:
        if t < deploy_time:
            return s_payload
        elif t <= deploy_time + opening_duration:
            # Linear interpolation during inflation
            fraction = (t - deploy_time) / opening_duration
            return s_payload + fraction * (s_chute - s_payload)
        else:
            return s_chute

    return drag_area


def calculate_state_derivative(
    t: float,
    state: np.ndarray,
    mass: float,
    drag_area: float,  # Now accepts the combined Cd * A
    env: SimulationEnvironment,
) -> np.ndarray:
    """Calculates the derivative of the state vector at time t"""
    v_payload = state[3:]
    v_relative = v_payload - env.wind_model(state[:3], t)
    v_rel_magnitude = np.linalg.norm(v_relative)

    # F_drag = -0.5 * rho * (Cd*A) * v_rel * v_vector
    drag_vector = -0.5 * env.air_density * drag_area * v_rel_magnitude * v_relative
    a_vector = (drag_vector / mass) + env.gravity_vector

    return np.concatenate([v_payload, a_vector])


def rk4_step(
    t: float,
    dt: float,
    state: np.ndarray,
    mass: float,
    drag_area_func: Callable[
        [float], float
    ],  # Function to get exact drag area at sub-steps
    env: SimulationEnvironment,
) -> np.ndarray:
    """Performs a single RK4 integration step with time-varying drag area."""
    # Calculate exact drag areas for RK4 intermediate steps
    da_t = drag_area_func(t)
    da_half = drag_area_func(t + dt / 2)
    da_next = drag_area_func(t + dt)

    k1 = calculate_state_derivative(t, state, mass, da_t, env)
    k2 = calculate_state_derivative(
        t + dt / 2, state + k1 * (dt / 2), mass, da_half, env
    )
    k3 = calculate_state_derivative(
        t + dt / 2, state + k2 * (dt / 2), mass, da_half, env
    )
    k4 = calculate_state_derivative(t + dt, state + k3 * dt, mass, da_next, env)

    next_state = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return next_state


def simulate_drop(
    initial_state: np.ndarray,
    mass: float,
    drag_area_func: Callable[[float], float],
    dt: float = 0.01,
    env: SimulationEnvironment | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Simulates the drop until it reaches the target elevation, returns time and state history."""
    if env is None:
        env = SimulationEnvironment()

    time_history = []
    state_history = []

    current_time = 0.0
    current_state = initial_state

    while current_state[2] > env.target_elevation:
        state_history.append(current_state)
        time_history.append(current_time)

        current_state = rk4_step(
            current_time, dt, current_state, mass, drag_area_func, env
        )
        current_time += dt

    return np.array(time_history), np.array(state_history)


class ShootingSolver:
    """class for finding release point using displacement"""

    def __init__(
        self, mass: float, drag_area_func, release_latency: float = 0.0
    ) -> None:
        self.mass = mass
        self.drag_area_func = drag_area_func
        self.release_latency = release_latency

    def calculate_release_point(
        self,
        target_position: np.ndarray,
        approach_altitude: float,
        approach_velocity: np.ndarray,
        env,
    ) -> np.ndarray:
        """Finds the optimal drop spot using a single forward-simulation (direct displacement)."""
        # find displacement
        test_initial_state = np.concatenate(
            [[0.0, 0.0], [approach_altitude], approach_velocity]
        )
        _, positions = simulate_drop(
            test_initial_state, self.mass, self.drag_area_func, dt=0.01, env=env
        )
        # get it
        final_test_position = positions[-1, :3]
        drift_vector = final_test_position[:2]
        # subtract displacement from target to get release point
        physical_drop_point = target_position[:2] - drift_vector
        # take into account latency
        optimal_command_position = physical_drop_point - (
            approach_velocity[:2] * self.release_latency
        )

        return optimal_command_position
