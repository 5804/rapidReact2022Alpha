// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.ShootHighGoalCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndFireCommandGroup extends SequentialCommandGroup {
  /** Creates a new AimAndFireCommandGroup. */
  public AimAndFireCommandGroup(LimelightSubsystem ls, DrivetrainSubsystem dts, ShooterSubsystem shoot, IntakeSubsystem is) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootHighGoalCommand(shoot), // the isFinished() method of this command returns false, might need to change this
      new AlignToGoalWithLimelightCommand(ls, dts),
      new FireShooterRoutine(shoot, is, ls, dts) // if this routine only shoots one ball, add the same line on the line below this one
    );
  }
}
