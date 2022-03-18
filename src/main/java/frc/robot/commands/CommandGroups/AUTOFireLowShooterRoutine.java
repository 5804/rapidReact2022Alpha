// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AUTOActivateAcceleratorCommand;
import frc.robot.commands.AUTOShootLowGoalCommand;
import frc.robot.commands.ConveyorToPositionCommand;
import frc.robot.commands.StopAcceleratorCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTOFireLowShooterRoutine extends SequentialCommandGroup {
  public AUTOFireLowShooterRoutine(ShooterSubsystem ss, IntakeSubsystem is) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AUTOShootLowGoalCommand(ss),
      new PrepareToShootCommandGroup(ss, is),
      new AUTOActivateAcceleratorCommand(ss),
      new WaitCommand(0.1),
      new ConveyorToPositionCommand(is, 5*2048, 1),
      new WaitCommand(0.3),
      new ConveyorToPositionCommand(is, 5*2048, 1),
      new StopShooterCommand(ss),
      new StopAcceleratorCommand(ss)

    );
  }
}
