#!/usr/bin/env python3
from dataclasses import dataclass, field

from numpy.ma import cos
import numpy as np

MOTOR_PULLEY_RADIUS = 9.400 # mm

## All coeffs are in mm/radian.
@dataclass
class FingerCoeffs:
    mcp_flex_coeff: float = 12.4912
    pip_coeff: float = 7.3211
    dip_coeff: float = 9.0000

    
    
    poly1_3_coeffs: list = field(default_factory=lambda: [12.4942043298569, 51.95335801017932, -33.501686142285934])
    poly2_4_coupled_coeffs: list = field(default_factory=lambda: [12.59493397149999, 20.133925694587813, -0.733600415686058, -1.9015760983591548])
    poly2_4_2order_coeffs: list = field(default_factory=lambda: [15.68463971095201, 9.056182182321907, -1.8049688179605574, -0.7608190837507814])
    poly2_coeffs_5: list = field(default_factory=lambda: [15.737611956982395, 8.93030725431122, -1.8297962629452305, -88.47527062453545, 175.80274440610984])
    poly2_6_decouple_coeffs: list = field(default_factory=lambda: [15.908062986986915, -10.414064668725372, 28.275491813684205, -1.901766617184215, 6.361024773505331, -7.7509787096326495])
    poly2_6_couple_coeffs: list = field(default_factory=lambda: [14.947770220619521, 8.839039802650348, -0.5137077382458715, -0.21434094448470242, -0.5098477412983466, -0.2763231710835051])
    poly3_coeffs_9: list = field(default_factory=lambda: [15.031096394280997, 0.684059974072479, 17.586977168564065, -0.590576437631079, -5.664242120595652, 3.264294504798293, -0.4799718038191859, 5.085342885587439, -4.379346947085984])
    poly3_coeffs_10: list = field(default_factory=lambda: [0.4548641049498947, 14.957259049914732, 5.445486625149704, 9.916020433745192, -0.48221573539230755, 6.520657130793484, -3.797711209623942, -0.5238509747835309, -3.040500961038191, 1.2360462038610134])
    
    
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
        

        """
        # This is the 1rd order polynomial model (3 parameters) - including all cross terms
        # formula (one order decoupled, 3 parameters):
        # A_deg = rad2deg( c1*m + c2*p + c3*d ) / R
        # where c1 = poly3_mcp_flex_coeff, c2 = poly3_pip_coeff, c3 = poly3_dip_coeff
        # return (
        #     self.finger_coeffs.poly3_mcp_flex_coeff * mcp_flex
        #     + self.finger_coeffs.poly3_pip_coeff * pip
        #     + self.finger_coeffs.poly3_dip_coeff * dip
        # ) / MOTOR_PULLEY_RADIUS   


        mcp_flex_rad = np.deg2rad(mcp_flex)
        pip_rad = np.deg2rad(pip)
        dip_rad = np.deg2rad(dip)

        # This is the 3rd order polynomial model (9 parameters) - including all cross terms        
        # formula (three order main terms, no cross terms, 9 parameters):
        # A_deg = rad2deg(
        #     a1*m + a2*p + a3*d
        #   + a4*m^2 + a5*p^2 + a6*d^2
        #   + a7*m^3 + a8*p^3 + a9*d^3
        # ) / R
        # coefficient vector a = poly3_coeffs_9[0..8]
        # return np.rad2deg(
        #     self.finger_coeffs.poly3_coeffs_9[0] * mcp_flex_rad
        #     + self.finger_coeffs.poly3_coeffs_9[1] * pip_rad
        #     + self.finger_coeffs.poly3_coeffs_9[2] * dip_rad
        #     + self.finger_coeffs.poly3_coeffs_9[3] * mcp_flex_rad**2
        #     + self.finger_coeffs.poly3_coeffs_9[4] * pip_rad**2
        #     + self.finger_coeffs.poly3_coeffs_9[5] * dip_rad**2
        #     + self.finger_coeffs.poly3_coeffs_9[6] * mcp_flex_rad**3
        #     + self.finger_coeffs.poly3_coeffs_9[7] * pip_rad**3
        #     + self.finger_coeffs.poly3_coeffs_9[8] * dip_rad**3
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 3rd order polynomial model (10 parameters) - including all cross terms
        # formula (three order main terms + constant term, 10 parameters):
        # A_deg = rad2deg(
        #     b0
        #   + b1*m + b2*p + b3*d
        #   + b4*m^2 + b5*p^2 + b6*d^2
        #   + b7*m^3 + b8*p^3 + b9*d^3
        # ) / R
        # coefficient vector b = poly3_coeffs_10[0..9]
        # return np.rad2deg(
        #     self.finger_coeffs.poly3_coeffs_10[0] 
        #     + self.finger_coeffs.poly3_coeffs_10[1] * mcp_flex_rad
        #     + self.finger_coeffs.poly3_coeffs_10[2] * pip_rad
        #     + self.finger_coeffs.poly3_coeffs_10[3] * dip_rad
        #     + self.finger_coeffs.poly3_coeffs_10[4] * mcp_flex_rad**2
        #     + self.finger_coeffs.poly3_coeffs_10[5] * pip_rad**2
        #     + self.finger_coeffs.poly3_coeffs_10[6] * dip_rad**2
        #     + self.finger_coeffs.poly3_coeffs_10[7] * mcp_flex_rad**3
        #     + self.finger_coeffs.poly3_coeffs_10[8] * pip_rad**3
        #     + self.finger_coeffs.poly3_coeffs_10[9] * dip_rad**3
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 2nd order polynomial model (6 parameters) - decoupled terms only
        # formula (two order main terms decoupled, 6 parameters):
        # A_deg = rad2deg(
        #     c1*m + c2*p + c3*d
        #   + c4*m^2 + c5*p^2 + c6*d^2
        # ) / R
        # coefficient vector c = poly2_6_decouple_coeffs[0..5]
        # return np.rad2deg(
        #     self.finger_coeffs.poly2_6_decouple_coeffs[0]*mcp_flex_rad
        #     + self.finger_coeffs.poly2_6_decouple_coeffs[1]*pip_rad
        #     + self.finger_coeffs.poly2_6_decouple_coeffs[2]*dip_rad
        #     + self.finger_coeffs.poly2_6_decouple_coeffs[3]*mcp_flex_rad**2
        #     + self.finger_coeffs.poly2_6_decouple_coeffs[4]*pip_rad**2
        #     + self.finger_coeffs.poly2_6_decouple_coeffs[5]*dip_rad**2
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 2nd order polynomial model (6 parameters) - coupled terms
        # formula (two order coupled, 6 parameters, including two order main terms and three order main terms):
        # A_deg = rad2deg(
        #     d1*m + d2*(p + d)
        #   + d3*m^2 + d4*(p^2 + d^2)
        #   + d5*m^3 + d6*(p^3 + d^3)
        # ) / R
        # coefficient vector d = poly2_6_couple_coeffs[0..5]
        # return np.rad2deg(
        #     self.finger_coeffs.poly2_6_couple_coeffs[0] * mcp_flex_rad + self.finger_coeffs.poly2_6_couple_coeffs[1] * (pip_rad + dip_rad) + 
        #     + self.finger_coeffs.poly2_6_couple_coeffs[2] * mcp_flex_rad**2 
        #     + self.finger_coeffs.poly2_6_couple_coeffs[3] * (pip_rad**2 + dip_rad**2)
        #     + self.finger_coeffs.poly2_6_couple_coeffs[4] * mcp_flex_rad**3 
        #     + self.finger_coeffs.poly2_6_couple_coeffs[5] * (pip_rad**3 + dip_rad**3)
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 2nd order polynomial model (5 parameters) - coupled terms
        # formula (two order coupled + cross terms, 5 parameters):
        # A_deg = rad2deg(
        #     e1*m + e2*(p + d)
        #   + e3*m^2 + e4*(p^2 + d^2) + e5*(p*d)
        # ) / R
        # coefficient vector e = poly2_coeffs_5[0..4]
        # return np.rad2deg(
        #     self.finger_coeffs.poly2_coeffs_5[0] * mcp_flex_rad + self.finger_coeffs.poly2_coeffs_5[1] * (pip_rad + dip_rad) + 
        #     + self.finger_coeffs.poly2_coeffs_5[2] * mcp_flex_rad**2 
        #     + self.finger_coeffs.poly2_coeffs_5[3] * (pip_rad**2 + dip_rad**2) + self.finger_coeffs.poly2_coeffs_5[4] * (pip_rad * dip_rad)
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 2nd order polynomial model (4 parameters) - 2nd order terms only
        # formula (two order main terms coupled, 4 parameters):
        # A_deg = rad2deg(
        #     f1*m + f2*(p + d)
        #   + f3*m^2 + f4*(p^2 + d^2)
        # ) / R
        # coefficient vector f = poly2_4_2order_coeffs[0..3]
        # return np.rad2deg(
        #     self.finger_coeffs.poly2_4_2order_coeffs[0] * mcp_flex_rad + self.finger_coeffs.poly2_4_2order_coeffs[1] * (pip_rad + dip_rad) + 
        #     + self.finger_coeffs.poly2_4_2order_coeffs[2] * mcp_flex_rad**2 
        #     + self.finger_coeffs.poly2_4_2order_coeffs[3] * (pip_rad**2 + dip_rad**2)
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 2nd order polynomial model (4 parameters) - coupled terms        
        # formula (two order coupled + cross terms, 4 parameters):
        # A_deg = rad2deg(
        #     g1*m + g2*p + g3*d + g4*(p*d)
        # ) / R
        # coefficient vector g = poly2_4_coupled_coeffs[0..3]
        # return np.rad2deg(
        #     self.finger_coeffs.poly2_4_coupled_coeffs[0] * mcp_flex_rad + self.finger_coeffs.poly2_4_coupled_coeffs[1] * pip_rad + self.finger_coeffs.poly2_4_coupled_coeffs[2] * dip_rad + self.finger_coeffs.poly2_4_coupled_coeffs[3] *(pip_rad * dip_rad)
        # ) / MOTOR_PULLEY_RADIUS

        # This is the 1st order polynomial model (3 parameters) - decoupled terms only
        # formula (one order decoupled, 3 parameters):
        # A_deg = rad2deg( h1*m + h2*p + h3*d ) / R
        return np.rad2deg(
            self.finger_coeffs.poly1_3_coeffs[0] * mcp_flex_rad + self.finger_coeffs.poly1_3_coeffs[1] * pip_rad + self.finger_coeffs.poly1_3_coeffs[2] * dip_rad
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
