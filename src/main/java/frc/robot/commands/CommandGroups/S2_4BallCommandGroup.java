// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AUTORunIntakeAndConveyorCommand;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S2_4BallCommandGroup extends SequentialCommandGroup {
 
  /** Creates a new S2_2BallCommandGroup. */
  public S2_4BallCommandGroup(DrivetrainSubsystem dts, ShooterSubsystem shooter, IntakeSubsystem is, LimelightSubsystem ls) {
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("4BallPartOne", 3, 3);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("4BallPartTwo", 5, 5);
    PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("4BallPartThree", 5, 5);

    addCommands(
      new ExtendIntakeCommand(is),
      new AUTORunIntakeAndConveyorCommand(is),
      new InstantCommand(()-> dts.resetOdometry(trajectory1.getInitialPose())),
      dts.createCommandForTrajectory(trajectory1).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new StopIntakeCommand(is),
      new AUTOFireShooterRoutine(shooter, is),
      new AUTORunIntakeAndConveyorCommand(is),
      dts.createCommandForTrajectory(trajectory2).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new WaitCommand(1.3),
      dts.createCommandForTrajectory(trajectory3).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new StopIntakeCommand(is),
      new AlignToGoalWithLimelightCommand(ls, dts),
      new AUTOFireShooterRoutine(shooter, is)
    );
  }
}

// new ExtendIntakeCommand(is),
// new AUTORunIntakeAndConveyorCommand(is),
// new InstantCommand(()-> dts.resetOdometry(trajectory1.getInitialPose())),
// dts.createCommandForTrajectory(trajectory1).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
// new StopIntakeCommand(is),
// new AUTOFireShooterRoutine(shooter, is),
// new AUTORunIntakeAndConveyorCommand(is),
// new InstantCommand(()-> dts.createCommandForTrajectory(trajectory2).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))),
// new WaitCommand(1.3),
// new InstantCommand(()-> dts.createCommandForTrajectory(trajectory3).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))),
// new StopIntakeCommand(is),
// new AlignToGoalWithLimelightCommand(ls, dts),
// new AUTOFireShooterRoutine(shooter, is)

// new ExtendIntakeCommand(is),
// new AUTORunIntakeAndConveyorCommand(is),
// new InstantCommand(()-> dts.resetOdometry(trajectory1.getInitialPose())),
// dts.createCommandForTrajectory(trajectory1).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
// new StopIntakeCommand(is),
// new AUTOFireShooterRoutine(shooter, is),
// new AUTORunIntakeAndConveyorCommand(is),
// new InstantCommand(()-> dts.resetOdometry(trajectory2.getInitialPose())),
// dts.createCommandForTrajectory(trajectory2).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
// new WaitCommand(1.3),
// new InstantCommand(()-> dts.resetOdometry(trajectory3.getInitialPose())),
// dts.createCommandForTrajectory(trajectory3).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
// new StopIntakeCommand(is),
// new AlignToGoalWithLimelightCommand(ls, dts),
// new AUTOFireShooterRoutine(shooter, is)