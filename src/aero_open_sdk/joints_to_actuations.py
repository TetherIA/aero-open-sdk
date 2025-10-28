#!/usr/bin/env python3
from dataclasses import dataclass, field
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from numpy.ma import cos
import numpy as np

MOTOR_PULLEY_RADIUS = 9.200 # mm

## All coeffs are in mm/radian.
@dataclass
class FingerCoeffs:
    poly3_coeffs_9: list = field(default_factory=lambda: 
    [8.658692384427493, 
    -0.6661011847153046, 
    10.55126683051879, 
    2.1310230819405525, 
    1.3069255487892184, 
    5.041519035608118, 
    -0.15444460100786384, 
    -0.5974463990747414, 
    -0.9558290953598836])

    def __post_init__(self):
        pass
@dataclass
class ThumbFlexCoeffs:
    cmc_abd_coeff: float = 2.5000
    cmc_flex_coeff: float = 12.4931


@dataclass
class ThumbIPCoeffs:
    cmc_abd_coeff: float = 2.5000
    cmc_flex_coeff: float = 2.5000
    mcp_coeff: float = 9.4372
    ip_coeff: float = 12.5000


class JointsToActuationsModel:
    """
    A model to convert joint positions to actuator movements for the Aero Hand Open.
    """

    def __init__(self) -> None:
        self.finger_coeffs = FingerCoeffs()
        self.thumb_flex_coeffs = ThumbFlexCoeffs()
        self.thumb_ip_coeffs = ThumbIPCoeffs()

    
    def finger_actuations(self, mcp_flex: float, pip: float, dip: float) -> float:
        """
        three-order polynomial finger actuator model (20 parameters, including all cross terms)
        The input is in radians
        """
        

        # This is the 3rd order polynomial model (9 parameters) - including all cross terms
        # 
        # model structure:
        # actuator_angle = (c0*mcp + c1*pip + c2*dip + c3*mcp² + c4*pip² + c5*dip² + c6*mcp³ + c7*pip³ + c8*dip³) / radius
        #
        # where:
        # - mcp, pip, dip: joint angles (radians)
        # - c0-c8: 9 polynomial coefficients
        # - radius: motor pulley radius
        #
        # Simplification strategy:
        # - PIP and DIP use the same coefficients (average value)
        # - first order term: c1*pip + c2*dip → (c1+c2)/2 * (pip + dip)
        # - second order term: c4*pip² + c5*dip² → (c4+c5)/2 * (pip² + dip²)  
        # - third order term: c7*pip³ + c8*dip³ → (c7+c8)/2 * (pip³ + dip³)
        #
        # final expression:
        # actuator = c0*mcp + (c1+c2)/2*pip + (c1+c2)/2*dip + c3*mcp² + (c4+c5)/2*pip² + (c4+c5)/2*dip² + c6*mcp³ + (c7+c8)/2*pip³ + (c7+c8)/2*dip³
        
        return np.rad2deg(
            self.finger_coeffs.poly3_coeffs_9[0] * mcp_flex
            + (self.finger_coeffs.poly3_coeffs_9[1]+self.finger_coeffs.poly3_coeffs_9[2])/2 * pip
            + (self.finger_coeffs.poly3_coeffs_9[1]+self.finger_coeffs.poly3_coeffs_9[2])/2 * dip
            + self.finger_coeffs.poly3_coeffs_9[3] * mcp_flex**2
            + (self.finger_coeffs.poly3_coeffs_9[4]+self.finger_coeffs.poly3_coeffs_9[5])/2 * pip**2
            + (self.finger_coeffs.poly3_coeffs_9[4]+self.finger_coeffs.poly3_coeffs_9[5])/2 * dip**2
            + self.finger_coeffs.poly3_coeffs_9[6] * mcp_flex**3
            + (self.finger_coeffs.poly3_coeffs_9[7]+self.finger_coeffs.poly3_coeffs_9[8])/2 * pip**3
            + (self.finger_coeffs.poly3_coeffs_9[7]+self.finger_coeffs.poly3_coeffs_9[8])/2 * dip**3
        ) / MOTOR_PULLEY_RADIUS




    def thumb_actuations(
        self, cmc_abd: float, cmc_flex: float, mcp: float, ip: float
    ) -> tuple[float, float, float]:
        ## Thumb CMC abduction and flexion are linear mappings.
        thumb_cmc_abd_actuation = cmc_abd

        ## Thumb CMC Tendon Linear Actuation Model
        ## thumb_flex_actuation = cmc_abd_coeff * cmc_abd + cmc_flex_coeff * cmc_flex
        thumb_cmc_flex_actuation = (
            self.thumb_flex_coeffs.cmc_abd_coeff * cmc_abd
            + self.thumb_flex_coeffs.cmc_flex_coeff * cmc_flex
        ) / MOTOR_PULLEY_RADIUS

        ## Thumb tendon based on linear modeling.
        ## thumb_tendon = cmc_abd_coeff * cmc_abd - cmc_flex_coeff * cmc_flex + mcp_coeff * mcp + ip_coeff * ip
        thumb_tendon_actuation = (
            self.thumb_ip_coeffs.cmc_abd_coeff * cmc_abd
            - self.thumb_ip_coeffs.cmc_flex_coeff * cmc_flex
            + self.thumb_ip_coeffs.mcp_coeff * mcp
            + self.thumb_ip_coeffs.ip_coeff * ip
        ) / MOTOR_PULLEY_RADIUS

        return thumb_cmc_abd_actuation, thumb_cmc_flex_actuation, thumb_tendon_actuation

    def hand_actuations(self, joint_positions: list[float]) -> list[float]:
        """
        Convert joint positions to actuator movements.
        Args:
            joint_positions (list): A list of 16 joint positions. (degrees)
        Returns:
            list: A list of 7 actuator movements. (degrees)
        """
        actuations = []
        actuations += self.thumb_actuations(*joint_positions[0:4])

        ## Loop over the four fingers and get the actuation values.
        for i in range(4):
            actuations.append(
                self.finger_actuations(*joint_positions[4 + i * 3 : 4 + (i + 1) * 3])
            )
        return actuations
