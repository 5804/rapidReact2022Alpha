// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AUTOActivateAcceleratorCommand;
import frc.robot.commands.AUTORunIntakeAndConveyorCommand;
import frc.robot.commands.ActivateAcceleratorCommand;
import frc.robot.commands.RunConveyorMotorCommand;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireShooterCommandGroup extends ParallelCommandGroup {
  /** Creates a new FireShooterCommandGroup. */
  public FireShooterCommandGroup(ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AUTOActivateAcceleratorCommand(shooter),
      new AUTORunIntakeAndConveyorCommand(intake)
      //FIXME ADD IN CONVEYOR BELT WHEN ADDED TO ROBOT
    );
  }
}
