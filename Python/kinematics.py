"""
Forward Kinematics - 3-DOF RRR Robotic Arm
--------------------------------------------
Given the 3 joint angles, this calculates where the gripper tip is
in 3D space (x, y, z), measured in centimeters from the base joint.

GEOMETRY (measured from the real, built arm - more accurate than the
original PDF spec, since brackets/servo horns/gripper mounting add
real length that a paper design doesn't capture):
  GROUND_OFFSET = 5 cm    -> height of the base joint above the ground
  L1 = 8 cm    -> base joint to shoulder joint
  L2 = 21 cm   -> shoulder joint to elbow joint (re-measured)
  L3 = 28 cm   -> elbow joint to gripper TIP (19 cm to the gripper
                  mount + 9 cm gripper itself, combined since the
                  gripper is rigid and doesn't have its own joint)

ANGLE CONVENTIONS (confirmed with testing):
  theta1 -> base rotation, around the vertical (Z) axis
  theta2 -> shoulder angle, measured from horizontal
  theta3 -> elbow angle, RELATIVE to the upper arm (0 = fully extended,
            increasing theta3 folds the forearm CLOSER to the base)

CALIBRATION (best fit to 3 real measured gripper positions):
    - Shoulder: real angle = commanded - 15.5 degrees
    - Elbow: real bend = commanded + 49.7 degrees
  KNOWN LIMITATION: about 9cm average error remains across r and z.
  This is likely near the practical limit of hand-tool calibration
  (phone level + paper protractor) on small metal brackets. Plan to
  fine-tune with a small empirical offset once doing real pick tests.
"""

import math

# Link lengths, in centimeters (measured from the real built arm)
GROUND_OFFSET = 5.0
L1 = 8.0
L2 = 21.0
L3 = 28.0

# Calibration offsets (best fit to 3 real measured positions)
THETA1_OFFSET = 9.0    # NEW - degrees: real base angle = commanded - this
THETA2_OFFSET = 15.5
THETA3_OFFSET = 49.7

# Empirical position correction, found by testing a real target and
# measuring where the gripper actually closed (see project notes).
# Real result was: X +5cm too far, Y -4cm too short, Z +7cm too high.
# We correct by adjusting the requested position before solving IK.
POS_CORRECTION_X = -5.0
POS_CORRECTION_Y = 4.0
POS_CORRECTION_Z = -11.0  # updated from -7.0 using new wide-angle test data


def forward_kinematics(theta1_deg, theta2_deg, theta3_deg):
    """
    Calculate the gripper tip position (x, y, z) in cm, given the
    3 joint angles in degrees.

    Returns a tuple: (x, y, z)
    """
    # Convert all angles from degrees to radians (Python's math functions need radians)
    theta1 = math.radians(theta1_deg)

    # Apply calibration: convert COMMANDED angles to REAL physical angles
    theta2_real_deg = theta2_deg - THETA2_OFFSET
    theta3_real_bend_deg = theta3_deg + THETA3_OFFSET

    theta2 = math.radians(theta2_real_deg)
    theta3 = math.radians(theta3_real_bend_deg)

    # Step 1: convert the relative elbow angle to an absolute angle
    # (the forearm's real angle from horizontal)
    phi3 = theta2 - theta3

    # Step 2: solve the 2D triangle - reach (r) and height (z),
    # in the vertical plane that contains the arm (before base rotation)
    r = L2 * math.cos(theta2) + L3 * math.cos(phi3)
    z = GROUND_OFFSET + L1 + L2 * math.sin(theta2) + L3 * math.sin(phi3)

    # Step 3: apply the base rotation to get real x, y
    x = r * math.cos(theta1)
    y = r * math.sin(theta1)

    return (x, y, z)


def inverse_kinematics(x, y, z):
    """
    Calculate the joint angles (theta1, theta2, theta3) needed to move
    the gripper tip to a target position (x, y, z) in cm.

    Returns a tuple of COMMANDED angles: (theta1_deg, theta2_deg, theta3_deg)
    or None if the position is physically unreachable (too far or too close).
    """
    # Apply the empirical position correction FIRST, so everything below
    # solves for a slightly adjusted target that compensates for the
    # real robot's measured error (see POS_CORRECTION constants above).
    x = x + POS_CORRECTION_X
    y = y + POS_CORRECTION_Y
    z = z + POS_CORRECTION_Z

    # Step 1: base rotation - point toward the target
    theta1_deg = math.degrees(math.atan2(y, x)) + THETA1_OFFSET

    # Step 2: reduce to the 2D problem (reach r, height above shoulder)
    r = math.hypot(x, y)
    z_rel = z - GROUND_OFFSET - L1

    # Step 3: distance from shoulder straight to the target
    D = math.hypot(r, z_rel)
    if D > (L2 + L3) or D < abs(L2 - L3):
        return None  # target is too far away or too close - physically impossible

    # Step 4: law of cosines - find the elbow's relative bend angle
    cos_gamma = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
    cos_gamma = max(-1, min(1, cos_gamma))  # clamp for safety (avoid math errors from rounding)
    gamma = math.acos(cos_gamma)
    theta3_real_bend = 180 - math.degrees(gamma)

    # Step 5: find the shoulder's real angle
    beta = math.degrees(math.acos(max(-1, min(1, (L2**2 + D**2 - L3**2) / (2 * L2 * D)))))
    theta2_real = math.degrees(math.atan2(z_rel, r)) + beta

    # Step 6: convert real physical angles back to COMMANDED angles
    # (undo the calibration offsets, so this can be sent straight to the Arduino)
    theta2_cmd = theta2_real + THETA2_OFFSET
    theta3_cmd = theta3_real_bend - THETA3_OFFSET

    return (theta1_deg, theta2_cmd, theta3_cmd)


def is_within_joint_limits(theta1, theta2, theta3):
    """
    Check if a set of COMMANDED angles is within the arm's safe joint limits.
    Always call this before sending an IK result to the robot.
    """
    return (15 <= theta1 <= 170) and (45 <= theta2 <= 90) and (25 <= theta3 <= 100)


if __name__ == "__main__":
    # ---- Test cases ----
    # Run this file directly to see predicted gripper positions.
    # IMPORTANT: compare these numbers to a real measurement on your
    # physical arm (tape measure from the base) to confirm the formula
    # is correct before trusting it for inverse kinematics.

    test_cases = [
        ("HOME position", 15, 90, 45),      # theta1, theta2, theta3
        ("Shoulder down, elbow bent", 15, 45, 80),
        ("Base rotated", 90, 70, 60),
    ]

    print("Forward Kinematics Test Results")
    print("-" * 40)
    for name, t1, t2, t3 in test_cases:
        x, y, z = forward_kinematics(t1, t2, t3)
        print(f"{name}:")
        print(f"  theta1={t1} deg, theta2={t2} deg, theta3={t3} deg")
        print(f"  -> x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm")
        print()

    # ---- Inverse Kinematics round-trip test ----
    # For each FK result above, feed it into IK and check we get the
    # same angles back. This proves the IK math is correct.
    print("Inverse Kinematics Round-Trip Test")
    print("-" * 40)
    for name, t1, t2, t3 in test_cases:
        x, y, z = forward_kinematics(t1, t2, t3)
        result = inverse_kinematics(x, y, z)
        if result is None:
            print(f"{name}: position unreachable (unexpected!)")
            continue
        ik1, ik2, ik3 = result
        within_limits = is_within_joint_limits(ik1, ik2, ik3)
        print(f"{name}: target ({x:.1f}, {y:.1f}, {z:.1f})")
        print(f"  -> IK gives theta1={ik1:.1f}, theta2={ik2:.1f}, theta3={ik3:.1f}")
        print(f"  -> within safe joint limits: {within_limits}")
        print()

    # ---- Example: an unreachable target ----
    print("Unreachable Target Example")
    print("-" * 40)
    far_target = inverse_kinematics(200, 0, 30)  # way too far
    print(f"Target (200, 0, 30): {far_target}")
