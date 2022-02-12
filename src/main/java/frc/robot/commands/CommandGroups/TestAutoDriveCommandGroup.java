// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsytem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoDriveCommandGroup extends SequentialCommandGroup {
  /** Creates a new TestAutoDriveCommand. */
  public TestAutoDriveCommandGroup(DrivetrainSubsystem dts, ShooterSubsytem shootSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToDistanceCommand(dts, 1.0),
                new TurnToAngleCommand(dts, 90, 1),
                new DriveToDistanceCommand(dts, 1.0),
                new TurnToAngleCommand(dts, 90, 1)
                );
  }
}
