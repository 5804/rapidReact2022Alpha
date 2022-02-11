// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestRotateCommand extends CommandBase {
  /** Creates a new TestRotateCommand. */

  private final double rotationSpeed;
  private final DrivetrainSubsystem drivetrainSubsystem;

  public TestRotateCommand(DrivetrainSubsystem dts, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = dts;
    rotationSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                0, // translation x supplier is 0
                0, // translation y supplier is 0
                rotationSpeed, // we only want the robot to rotate, so this value is nonzero
                new Rotation2d(0) // i dont know why we need this line...
                )
        );
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
