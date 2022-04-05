// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AUTOActivateAcceleratorCommand;
import frc.robot.commands.AUTOShootHighGoalCommand;
import frc.robot.commands.ActivateAcceleratorCommand;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.ConveyorToPositionCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.RunConveyorMotorCommand;
import frc.robot.commands.ShootHighGoalCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireShooterRoutine extends SequentialCommandGroup {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  /** Creates a new AUTOFireShooterRoutine. */
  public FireShooterRoutine(ShooterSubsystem ss, IntakeSubsystem is, LimelightSubsystem ls, DrivetrainSubsystem dts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StopIntakeCommand(is),
      new ExtendIntakeCommand(is),
      new AlignToGoalWithLimelightCommand(ls, dts),
      new PrepareToShootCommandGroup(ss, is),
      new ActivateAcceleratorCommand(ss),
      new WaitCommand(0.1),
      new ConveyorToPositionCommand(is, 4*2048, 1),
      new WaitCommand(0.4),
      new ConveyorToPositionCommand(is, 5*2048, 1)
      
    );
    shooterSubsystem = ss;
    intakeSubsystem = is;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAccelerator();
    intakeSubsystem.stopConveyorMotor();
  }
}
