// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ActivateAcceleratorCommand extends CommandBase {
  /** Creates a new ActivateAcceleratorCommand. */
  private final ShooterSubsystem shooterSubsystem;
  
  public ActivateAcceleratorCommand(ShooterSubsystem shooter) {
    shooterSubsystem = shooter; 
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.runAccelerator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooterSubsystem.stopAccelerator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterSubsystem.accelerator.getMotorOutputPercent() > .95) {
      return true;
    } else {
        return false; 
    }
  }
}
