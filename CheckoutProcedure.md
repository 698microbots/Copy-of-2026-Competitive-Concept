# Robot Checkout Procedure

This document outlines the steps to verify the functionality of the robot's hardware and software systems.

## Initial Conditions
1.  **Safety:** Ensure the robot is on blocks (for drivetrain tests) or in a clear area.
2.  **Power:** Battery is secured, connected, and has >12.5V.
3.  **Radio:** Robot is linked to the Driver Station (DS).
4.  **Controllers:** Swerve Driver (Port 0) and Operator (Port 1) are connected and assigned.
5.  **Code:** Code is deployed and the DS shows "Robot Code" green.
6.  **Starting Position:** Intake and Hanger are in their physical "stowed" or "starting" positions.

---

## 1. Drivetrain & Vision (Swerve Driver - Port 0)

| Test | Action | Expected Behavior |
| :--- | :--- | :--- |
| **Field-Centric Reset** | Press **Back** button. | Current robot heading is set as "Forward" (0 degrees). |
| **Manual Translation** | Move **Left Stick** (from Operator controller). | Robot moves in the direction the stick is pushed, relative to the field. |
| **Manual Rotation** | Move **Right Stick** X-axis (from Operator controller). | Robot rotates in place. |
| **Vision Alignment** | Place an AprilTag in view and hold **Y**. | Robot automatically rotates to face the tag and adjusts position to center it. |

## 2. Intake & Internal Transport (Operator - Port 1)

| Test | Action | Expected Behavior |
| :--- | :--- | :--- |
| **Intake Rollers** | Hold **Left Trigger**. | Intake rollers spin inward. |
| **Intake Stop** | Release **Left Trigger**. | Intake rollers stop immediately. |
| **Floor Feed** | Hold **X Button**. | Floor belt/mechanism moves to transport game piece toward the feeder. |
| **Feeder Spin** | Hold **Right Trigger**. | Feeder rollers spin to move the game piece into the shooter. |

## 3. Shooter & Hood (Operator - Port 1)

| Test | Action | Expected Behavior |
| :--- | :--- | :--- |
| **Shooter Spin-Up** | Hold **Left Bumper**. | Shooter wheels spin up to 1000 RPM (audible whine). |
| **Hood Position** | Press **B Button**. | Hood moves to position 0.9 (check for smooth movement). |
| **Full Shoot Cycle** | Press **A Button**. | Shooter spins up to 2000 RPM, then the feeder and floor run sequentially to fire a piece. |

## 4. Hanger / Climber (Operator - Port 1)

| Test | Action | Expected Behavior |
| :--- | :--- | :--- |
| **Hanger Deploy** | Press **D-pad Up**. | Hanger mechanism extends to the "HANGING" position. |
| **Hanger Retract** | Press **D-pad Down**. | Hanger mechanism retracts to the "HUNG" (pulled up) position. |

---

## Post-Checkout Verification
*   [ ] Check for any loose bolts or wires after mechanism movement.
*   [ ] Verify no "CAN Timeout" or "Motor Overheating" errors in the DS log.
*   [ ] Ensure Limelight stream is visible on the Dashboard.
