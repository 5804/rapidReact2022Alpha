// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

public class AutonomousTestCommand extends CommandBase {
  /** Creates a new AutonomousTestCommand. */

  private final DrivetrainSubsystem drivetrainSubsystem; 


  public AutonomousTestCommand(DrivetrainSubsystem dts) {

    drivetrainSubsystem = dts;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
          kMaxSpeedMetersPerSecond,
          kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(drivetrainSubsystem.m_kinematics);

    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        config);

    var thetaController =
      new ProfiledPIDController(
          kPThetaController, 0, kDThetaController, kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            drivetrainSubsystem::getPose, // Functional interface to feed supplier
            drivetrainSubsystem.m_kinematics,

            // Position controllers
            new PIDController(kPXController, 0, 0),
            new PIDController(kPYController, 0, 0),
            thetaController,
            drivetrainSubsystem::setModuleStates,
            drivetrainSubsystem);
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
