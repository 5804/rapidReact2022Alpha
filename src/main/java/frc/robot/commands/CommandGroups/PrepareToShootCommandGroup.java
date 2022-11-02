// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcceleratorToPositionCommand;
import frc.robot.commands.ConveyorToPositionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareToShootCommandGroup extends SequentialCommandGroup {
  /** Creates a new PrepareToShootCommandGroup. */
  public PrepareToShootCommandGroup(ShooterSubsystem ss, IntakeSubsystem is) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AcceleratorToPositionCommand(ss, 4092, -1)
      // new ConveyorToPositionCommand(is, 32, -1)
    );
  }
}
