// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// For this command, do we want only the motors for the intake and conveyor running, or do we want the intake pistons to go out as well?

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeAndConveyor extends CommandBase {
  /** Creates a new RunIntakeAndConveyor. */

  private final IntakeSubsystem intakeSubsystem;

  public RunIntakeAndConveyor(IntakeSubsystem is) {
    // Use addRequirements() here to declare subsystem dependencies.

    intakeSubsystem = is;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runIntakeMotor();
    intakeSubsystem.activateIntakePiston();
    intakeSubsystem.runConveyorMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopConveyorMotor();
    intakeSubsystem.deactivateIntakePiston();
    intakeSubsystem.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
