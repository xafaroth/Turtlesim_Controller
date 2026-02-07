import numpy as np
from typing import Callable

def uniform_motion(current: float, std: float) -> float:
    return np.random.uniform(-3.0, 3.0)

def cauchy_motion(current: float, std: float) -> float:
    return np.random.standard_cauchy() * std

def poisson_motion(current: float, std: float) -> float:
    return np.random.choice([-1.5, -0.5, 0, 0.5, 1.5]) * std

def exponential_motion(current: float, std: float) -> float:
    return np.random.exponential(std) * np.random.choice([-1, 1])

def brownian_motion(current: float, std: float) -> float:
    return np.clip(current + np.random.randn() * std, -3.0, 3.0)

def gaussian_motion(current: float, std: float) -> float:
    return np.random.randn() * std

MOTION_GENERATORS: dict[str, Callable[[float, float], float]] = {
    "Uniform": uniform_motion,
    "Cauchy": cauchy_motion,
    "Poisson": poisson_motion,
    "Exponential": exponential_motion,
    "Brownian": brownian_motion,
    "Gaussian": gaussian_motion
}

PEN_COLORS: dict[str, tuple[int, int, int]] = {
    "Red": (255, 0, 0),
    "Blue": (0, 0, 255),
    "Green": (0, 255, 0),
    "Yellow": (255, 255, 0),
    "Cyan": (0, 255, 255),
    "Magenta": (255, 0, 255),
    "White": (255, 255, 255),
    "Orange": (255, 165, 0),
    "Purple": (128, 0, 128),
    "Pink": (255, 192, 203),
    "Brown": (165, 42, 42),
    "Gray": (128, 128, 128),
}