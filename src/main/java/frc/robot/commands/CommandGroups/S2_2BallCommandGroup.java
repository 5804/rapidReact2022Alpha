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
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S2_2BallCommandGroup extends SequentialCommandGroup {
 
  /** Creates a new S2_2BallCommandGroup. */
  public S2_2BallCommandGroup(DrivetrainSubsystem dts, ShooterSubsystem shooter, IntakeSubsystem is) {
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("2_StoBtoSH", 3, 3);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("SHtoT2", 3, 3);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ExtendIntakeCommand(is),
      new AUTORunIntakeAndConveyorCommand(is),
      new InstantCommand(()-> dts.resetOdometry(trajectory1.getInitialPose())),
      dts.createCommandForTrajectory(trajectory1).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
      new InstantCommand(()-> dts.resetOdometry(trajectory2.getInitialPose())),
      dts.createCommandForTrajectory(trajectory2).andThen(() -> dts.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))

    );
  }
}
