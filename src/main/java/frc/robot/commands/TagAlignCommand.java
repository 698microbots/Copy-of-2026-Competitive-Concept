// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagAlignCommand extends Command {
  /** Creates a new TagAlignCommand. */
  private Swerve swerve = new Swerve();
  private Limelight limelight = new Limelight("limelight");
  private final PIDController pidControllerX = new PIDController(.35, 0.0005, .0000095);
  private final PIDController pidControllerOmega = new PIDController(.06, .0005, 0.0000095);
  private final PIDController pidControllerY = new PIDController(.3, 0.0, 0);
  private double initalPositionY = 0;
  private Supplier<Double> x, omega;

  public TagAlignCommand(Limelight limelight, Swerve swerve, Supplier<Double> x, Supplier<Double> omega) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.x = x;
    this.omega = omega;
    addRequirements(limelight);
    addRequirements(swerve); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //if there are any visible targets 
      if (limelight.getHasTargets()){
        
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

