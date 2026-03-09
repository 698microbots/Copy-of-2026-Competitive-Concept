# 2026CompetitiveConcept

This repository contains the code used for the WestCoast Products 2026 [Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts).

The project is based on one of CTRE's [Phoenix 6 example projects](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithChoreo). It uses WPILib [command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) to manage robot subsystems and actions, a [Limelight](https://limelightvision.io/) for vision, and [Choreo](https://choreo.autos/) for autonomous path following.

## Gamepad Bindings

### Swerve Driver (Port 0)
*   **Y Button (Hold):** Vision-assisted drive (Aligns with AprilTag using Limelight).
*   **Back Button:** Resets field-centric heading.

### Operator / Secondary Driver (Port 1)
*   **Left Stick:** Manual translation (X and Y movement).
*   **Right Stick (X-axis):** Manual rotation.
*   **A Button:** `shootAndFeed` (Composite command to fire game pieces).
*   **B Button:** Sets Hood position to 0.9.
*   **X Button (Hold):** Runs the Floor feed motor.
*   **Left Bumper (Hold):** Spins up the Shooter to 1000 RPM.
*   **Left Trigger (Hold):** Spins the Intake rollers (stops when released).
*   **Right Trigger (Hold):** Spins the Feeder motor.
*   **D-pad Up:** Moves Hanger to `HANGING` position.
*   **D-pad Down:** Moves Hanger to `HUNG` position.
