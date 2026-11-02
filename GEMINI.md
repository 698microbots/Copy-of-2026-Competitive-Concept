# GEMINI.md - Project Context

This project is a WPILib 2026 Java robot application for a "Competitive Concept" robot by West Coast Products. It is built using the command-based programming paradigm and integrates advanced features like Swerve drive, Choreo trajectories, and Limelight vision.

## Project Overview

*   **Framework:** WPILib (Java) 2026
*   **Architecture:** Command-Based Programming
*   **Drive System:** CTRE Phoenix 6 Swerve Drivetrain (via `TunerConstants`)
*   **Vision:** Limelight for AprilTag localization and targeting
*   **Autonomous:** Choreo for path planning and following
*   **Key Subsystems:**
    *   `Swerve`: Field-centric drivetrain with vision integration.
    *   `Intake`: Game piece collection.
    *   `Floor` & `Feeder`: Internal transport and indexing.
    *   `Shooter` & `Hood`: Game piece launching with adjustable angle.
    *   `Hanger`: Climbing mechanism.

## Building and Running

*   **Build Project:** `./gradlew build`
*   **Deploy to Robot:** `./gradlew deploy`
*   **Run Simulation:** `./gradlew simulateJava`
*   **Run Tests:** `./gradlew test`
*   **Clean Build:** `./gradlew clean`

## Development Conventions

*   **Command Bindings:** Most trigger-to-command mappings are defined in `RobotContainer.configureBindings()`.
*   **Constants:** Robot-wide numerical constants are located in `src/main/java/frc/robot/Constants.java` and `TunerConstants.java` (for swerve).
*   **Vision Integration:** Vision measurements from the Limelight are periodically added to the Swerve odometry via `Swerve.addVisionMeasurement`.
*   **Field Centricity:** The robot uses field-centric driving by default, with alliance-aware perspective handling in `Swerve.periodic()`.
*   **Trajectory Following:** Autonomous paths are managed via `Choreo` and the `AutoFactory` created in `Swerve.java`.

## Key Files and Directories

*   `src/main/java/frc/robot/RobotContainer.java`: The central location for robot configuration and command orchestration.
*   `src/main/java/frc/robot/subsystems/`: Contains all hardware-specific subsystem logic.
*   `src/main/java/frc/robot/commands/`: Contains command compositions and autonomous routines.
*   `src/main/deploy/choreo/`: Contains `.traj` files for autonomous paths.
*   `vendordeps/`: Vendor library definitions (Phoenix 6, ChoreoLib, WPILib).

## Troubleshooting

*   **Deployment Issues:** Ensure the robot is powered on and connected via USB, Ethernet, or Wi-Fi. Verify the team number in `.wpilib/wpilib_preferences.json`.
*   **Swerve Calibration:** Drivetrain-specific constants are generated via the CTRE Tuner and stored in `TunerConstants.java`.



