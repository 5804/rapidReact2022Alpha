// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleCommand extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem;
  public double getGyroscopeRotation;
  public double numberOfDegrees;
  public double direction;
  public double angleInit;


  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DrivetrainSubsystem dts, double degrees, int dir) { //Clockwise is 1, Counter clockwise is -1
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = dts;
    numberOfDegrees = degrees;
    direction = dir;
    addRequirements(drivetrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      angleInit = drivetrainSubsystem.getGyroscopeRotation().getDegrees();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (direction > 0) {
      drivetrainSubsystem.rotate(0.2);
    } else {
      drivetrainSubsystem.rotate(-0.2);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(drivetrainSubsystem.getGyroscopeRotation().getDegrees() - angleInit) > 0) {
      return true;
    } else {
      return false;
    }
  }
}
