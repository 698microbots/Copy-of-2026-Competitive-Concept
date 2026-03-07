// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.commands.TagAlignCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    protected final Limelight limelight = new Limelight("ll");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    //Swerve driver on port 0 in Driver Station:
    private final CommandXboxController swerveDriver = new CommandXboxController(0);

    //Secondary driver on port 1 in Driver Station:
    private final CommandXboxController driver2 = new CommandXboxController(1);

    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver2.getLeftY(),
        () -> -driver2.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        //Original default commands at beginning of match:
        // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
        //     .onTrue(intake.homingCommand())  //puts intake in starting position
        //     .onTrue(hanger.homingCommand()); //puts hanger/climber in starting position

        //Default controller bindings:
        // driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        //driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        //driver.leftTrigger().whileTrue(intake.intakeCommand());  
        //driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        
        //Default hanger bindings:
        driver2.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING)); //povUp is the up arrow on D-pad
        driver2.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG)); //povDown is down arrow on D-pad

        //Intake test (runs the rollers when pressed and stops when not pressed):
        driver2.leftTrigger().whileTrue(intake.spin());
        driver2.leftTrigger().whileFalse(intake.stop());

        //Feeder test (feeder keeps running after not pressed):
        driver2.rightTrigger().whileTrue(feeder.spin());
       //Caused feeder motor to slip and feeder stopped:
       // driver.rightTrigger().whileFalse(feeder.stop());

       //Shooter test:
        driver2.leftBumper().whileTrue(shooter.spinUpCommand(1000));

        //Floor test:
        driver2.x().whileTrue(floor.feedCommand());

        //Hood test (position between 0.0 and 1.0):
        driver2.b().onTrue(hood.positionCommand(0.9));

        //Shoot manually test:
        //driver.a().onTrue(subsystemCommands.shootManually());
        driver2.a().onTrue(subsystemCommands.shootAndFeed());

    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver2.getLeftY(), 
            () -> -driver2.getLeftX(), 
            () -> -driver2.getRightX()
        );
        
        swerve.setDefaultCommand(manualDriveCommand);

        //Default 'a' 'b' 'x' 'y' button bindings (rotation commands):
        //driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        //driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        //driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        // driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
       
       //Track and align with aprilTag:
       swerveDriver.y().whileTrue(new ManualDriveCommand(
            swerve, 
            () -> LimelightHelpers.getTY("limelight") * 0.1, 
            () -> -driver2.getLeftX(), // forward and backward motion is controlled by driver
            () -> LimelightHelpers.getTX("limelight")*.03
        ));

        //P1 reset field-centric heading:
        swerveDriver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));

    }


    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }

}
