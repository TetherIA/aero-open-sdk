from dataclasses import dataclass

@dataclass(frozen=True)
class AeroHandConstants:
    ## Joints (16)
    joint_names: tuple[str, ...] = (
        "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip", ## 4 Joints in thumb
        "index_mcp_flex", "index_pip", "index_dip",               ## 3 Joints in index
        "middle_mcp_flex", "middle_pip", "middle_dip",            ## 3 Joints in middle
        "ring_mcp_flex", "ring_pip", "ring_dip",               ## 3 Joints in ring
        "pinky_mcp_flex", "pinky_pip", "pinky_dip"                ## 3 Joints in pinky
    )

    joint_lower_limits: tuple[float, ...] = (
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    )
    joint_upper_limits: tuple[float, ...] = (
        100.0, 55.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0
    )

    ## Actuations (7)
    actuation_names: tuple[str, ...] = (
        "thumb_cmc_abd_act", "thumb_cmc_flex_act", "thumb_tendon_act",      ## 3 Actuators in thumb
        "index_tendon_act", "middle_tendon_act", "ring_tendon_act", "pinky_tendon_act" ## One actuator per finger
    )

    actuation_lower_limits: tuple[float, ...] = (
        0.0, 0.0, -27.7778, 0.0, 0.0, 0.0, 0.0
    )
    actuation_upper_limits: tuple[float, ...] = (
        100.0, 131.8906, 274.9275, 288.1603, 288.1603, 288.1603, 288.1603
    )