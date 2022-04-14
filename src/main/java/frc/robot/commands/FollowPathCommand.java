// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

public class FollowPathCommand extends PPSwerveControllerCommand {
  /** Creates a new GenericAutonomousCommand. */
  private final DrivetrainSubsystem driveTrainSubsystem;

  public FollowPathCommand(DrivetrainSubsystem driveTrainSubsystem, PathPlannerTrajectory path) {
    // Use addRequirements() here to declare subsystem dependencies.

    

    super(
      path,
      driveTrainSubsystem::getPose, // Functional interface to feed supplier
      driveTrainSubsystem.m_kinematics,

      // Position controllers
      new PIDController(kPXController, 0, 0),
      new PIDController(kPYController, 0, 0),
      driveTrainSubsystem.thetaController,
      driveTrainSubsystem::setModuleStates,
      driveTrainSubsystem
    );
    
    driveTrainSubsystem.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    driveTrainSubsystem.resetOdometry(path.getInitialPose());

    //FIXME: Does addrequirements have to be called for the driveTrainSubsystem?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveTrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
