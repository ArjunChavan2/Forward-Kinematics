"""Quintic (5th-order) polynomial trajectories for one joint: q(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5.

Used by joint_state_publisher when USE_QUINTIC is on. Pure math, no registry or robot here.
"""
from __future__ import annotations


def coefficients(q0: float, v0: float, a0: float, q1: float, v1: float, a1: float, T: float) -> list[float]:
    """[c0, c1, c2, c3, c4, c5] of the quintic that starts at position q0, velocity v0, acceleration a0 at t = 0
    and ends at q1, v1, a1 at t = T (T > 0).

    Six boundary conditions, six unknowns: q(0) = q0, q'(0) = v0, q''(0) = a0, q(T) = q1, q'(T) = v1, q''(T) = a1.
    """
    #t = 0:
    c0 = q0
    c1 = v0
    c2 = a0 / 2
    
    #t = T:
    R0 = q1 - (c0 + c1 * T + c2 * T**2)
    R1 = v1 - (c1 + 2 * c2 * T)
    R2 = a1 - (2 * c2)
    
    c3 = (10 * R0 - 4*(R1 * T) + 0.5*(R2*T**2)) / T**3
    c4 = (-15 * R0 + 7*(R1 * T) - (R2*T**2)) / T**4
    c5 = (6 * R0 - 3*(R1 * T) + 0.5*(R2*T**2)) / T**5
    
    return [c0, c1, c2, c3, c4, c5]

'''
    q1 - (c0 + c1 * T + c2 * T**2) = c3 * T**3 + c4 * T**4 + c5 * T**5
    v1 - (c1 + 2 * c2 * T)         = 3 * c3 * T**2 + 4* c4 * T**3 + 5 * c5 * T**4
    a1 - (2 * c2)                  = 6 * c3 * T + 12 * c4 * T**2 + 20 * c5 * T**3
'''

def evaluate(c: list[float], t: float) -> tuple[float, float, float]:
    """(position, velocity, acceleration) of the quintic with coefficients c at time t."""
    c0, c1, c2, c3, c4, c5 = c
    q = (c0 + 
         c1 * t + 
         c2 * t**2 + 
         c3 * t**3 + 
         c4 * t**4 + 
         c5 * t**5)
    v = (1 * c1 + 
         2 * c2 * t + 
         3 * c3 * t**2 + 
         4* c4 * t**3 + 
         5 * c5 * t**4)
    a = (2 * c2 + 
         6 * c3 * t + 
         12 * c4 * t**2 + 
         20 * c5 * t**3)
    return (q, v, a)
