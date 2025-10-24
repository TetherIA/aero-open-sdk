#!/usr/bin/env python3
from dataclasses import dataclass, field

from numpy.ma import cos
import numpy as np

MOTOR_PULLEY_RADIUS = 9.200 # mm

## All coeffs are in mm/radian.
@dataclass
class FingerCoeffs:
    mcp_flex_coeff: float = 12.4912
    pip_coeff: float = 7.3211
    dip_coeff: float = 9.0000
    
    # three-order polynomial coefficients (20 parameters) - including all cross terms
    # constant term (1) + first order term (3) + second order term (6) + third order term (10) = 20 parameters
    poly3_coeffs: list = field(default_factory=lambda: [
        -0.014953950415335525, 
        7.336552965759659, 
        16.67611960363351, 
        -4.063411240901369, 
        6.851915455964726, 
        754.027870128995, 
        730.6763106562287, 
        20.188252233854158, 
        -29.793791754141008, 
        -1475.4817856158984, 
        -2.085051927518757, 
        1039.3669940710247, 
        -1309.4906354895002, 
        -2.50357531852256, 
        5.535894692936951, 
        -186.06064807020098, 
        -131.16328104297074, 
        -3364.2768864665045, 
        3631.465463946007, 
        318.0904430022246])
    
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
        ## Finger Tendon Linear Actuation Model
        ## finger_tendon = mcp_flex_coeff * mcp_flex + pip_coeff * pip + dip_coeff * dip
        return (
            self.finger_coeffs.mcp_flex_coeff * mcp_flex
            + self.finger_coeffs.pip_coeff * pip
            + self.finger_coeffs.dip_coeff * dip
        ) / MOTOR_PULLEY_RADIUS


    def finger_actuations_complex(self, mcp_flex: float, pip: float, dip: float) -> float:
        """
        three-order polynomial finger actuator model (20 parameters, including all cross terms)
        
        three-order polynomial includes all possible terms:
        - constant term: c0
        - first order term: c1*mcp + c2*pip + c3*dip
        - second order term: c4*mcp^2 + c5*pip^2 + c6*dip^2 + c7*mcp*pip + c8*mcp*dip + c9*pip*dip
        - third order term: c10*mcp^3 + c11*pip^3 + c12*dip^3 + c13*mcp^2*pip + c14*mcp^2*dip + 
                  c15*mcp*pip^2 + c16*mcp*dip^2 + c17*pip^2*dip + c18*pip*dip^2 + c19*mcp*pip*dip
        
        Args:
            mcp_flex: MCP joint angle (degrees)
            pip: PIP joint angle (degrees) 
            dip: DIP joint angle (degrees)
        Returns:
            actuator angle (degrees)
        """
        # convert degrees to radians
        mcp_rad = np.deg2rad(mcp_flex)
        pip_rad = np.deg2rad(pip)
        dip_rad = np.deg2rad(dip)
        
        # three-order polynomial coefficients (20 parameters)
        coeffs = self.finger_coeffs.poly3_coeffs
        
        # calculate three-order polynomial (including all cross terms)
        result = (
            coeffs[0] +  # constant term
            coeffs[1] * mcp_rad + coeffs[2] * pip_rad + coeffs[3] * dip_rad +  # first order term
            coeffs[4] * mcp_rad**2 + coeffs[5] * pip_rad**2 + coeffs[6] * dip_rad**2 +  # second order term
            coeffs[7] * mcp_rad * pip_rad + coeffs[8] * mcp_rad * dip_rad + coeffs[9] * pip_rad * dip_rad +  # second order cross term
            coeffs[10] * mcp_rad**3 + coeffs[11] * pip_rad**3 + coeffs[12] * dip_rad**3 +  # third order term
            coeffs[13] * mcp_rad**2 * pip_rad + coeffs[14] * mcp_rad**2 * dip_rad +  # third order cross term
            coeffs[15] * mcp_rad * pip_rad**2 + coeffs[16] * mcp_rad * dip_rad**2 +  # third order cross term
            coeffs[17] * pip_rad**2 * dip_rad + coeffs[18] * pip_rad * dip_rad**2 +  # third order cross term
            coeffs[19] * mcp_rad * pip_rad * dip_rad  # third order cross term
        )
        
        # convert radians to degrees and divide by motor pulley radius
        return np.rad2deg(result) / MOTOR_PULLEY_RADIUS

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
