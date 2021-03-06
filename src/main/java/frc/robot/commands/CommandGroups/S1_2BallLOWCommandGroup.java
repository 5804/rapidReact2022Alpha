// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AUTORunIntakeAndConveyorCommand;
import frc.robot.commands.AUTOShootHighGoalCommand;
import frc.robot.commands.AUTOShootLowGoalCommand;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.ShootHighGoalCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S1_2BallLOWCommandGroup extends SequentialCommandGroup {
  /** Creates a new S1_2BallCommandGroup. */
  public S1_2BallLOWCommandGroup(DrivetrainSubsystem dts, ShooterSubsystem shooter, IntakeSubsystem is, LimelightSubsystem ls) { //TODO IMPLIMENT SHOOTER
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Start1ToA", 3, 1);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("AtoLow", 3, 1);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ExtendIntakeCommand(is),
      new AUTORunIntakeAndConveyorCommand(is),
      new InstantCommand(()-> dts.resetOdometry(trajectory1.getInitialPose())),
      dts.createCommandForTrajectory(trajectory1).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new StopIntakeCommand(is),
      new InstantCommand(()-> dts.resetOdometry(trajectory2.getInitialPose())),
      dts.createCommandForTrajectory(trajectory2).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new AUTOFireLowShooterRoutine(shooter, is)
    );

  }
}
