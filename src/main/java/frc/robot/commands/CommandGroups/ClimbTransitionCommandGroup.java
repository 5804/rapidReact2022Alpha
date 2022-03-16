// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActivateBottomPistonCommand;
import frc.robot.commands.ActivateTopPistonCommand;
import frc.robot.commands.DeactivateBottomPistonCommand;
import frc.robot.commands.DeactivateHookPistonCommand;
import frc.robot.commands.DeactivateTopPistonCommand;
import frc.robot.commands.StopWinchCommand;
import frc.robot.commands.WinchOutForSlack;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbTransitionCommandGroup extends SequentialCommandGroup {
  /** Creates a new ClimbTransitionCommandGroup. */
  public ClimbTransitionCommandGroup(ClimberSubsystem css) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new DeactivateTopPistonCommand(css),
    new DeactivateBottomPistonCommand(css),
    new WaitCommand(0.125),
    new DeactivateHookPistonCommand(css),
    new WaitCommand(0.2),
    new WinchOutForSlack(css),
    new WaitCommand(0.1),
    new ActivateTopPistonCommand(css),
    new WaitCommand(0.25),
    new DeactivateTopPistonCommand(css),
    new WaitCommand(0.125),
    new ActivateBottomPistonCommand(css),
    new StopWinchCommand(css)
    );
  }
}
