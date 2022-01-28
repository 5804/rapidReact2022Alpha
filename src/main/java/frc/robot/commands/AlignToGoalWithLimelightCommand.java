/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToGoalWithLimelightCommand extends CommandBase {

  private final LimelightSubsystem limelightSubsystem;
  private final DrivetrainSubsystem driveTrainSubsystem;
  private double steerCommand;

  /**
   * Creates a new AlignWithLimelightCommand.
   */
  public AlignToGoalWithLimelightCommand(LimelightSubsystem ls, DrivetrainSubsystem dts) {

    limelightSubsystem = ls;
    driveTrainSubsystem = dts;
    addRequirements(limelightSubsystem, driveTrainSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    steerCommand = limelightSubsystem.getSteeringValue();

    // When the limelight button is pressed, this code will run:
    // The code below will tell the drivetrain to NOT translate, which is why both of the translations are set to 0
    // But it will tell the robot to rotate at a given speed based on the reading from the limelight
    driveTrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0, // translation x supplier is 0
        0, // translation y supplier is 0
        steerCommand, // we only want the robot to rotate, so this value is nonzero
        new Rotation2d(0) // i dont know why we need this line...
      )
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
