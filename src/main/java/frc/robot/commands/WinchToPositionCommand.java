// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WinchToPositionCommand extends CommandBase {
  /** Creates a new WinchToPositionCommand. */


  ClimberSubsystem climberSubsystem;
  public double clicksNeeded;
  private double clicksTravelled = 0;
  public WinchToPositionCommand(double clicks, ClimberSubsystem climber) {
    clicksNeeded = clicks;
    climberSubsystem = climber;
    clicksTravelled = 0;

    addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.resetWinchEncoders();
    while (climberSubsystem.getWinchEncoders() != 0) {
      climberSubsystem.resetWinchEncoders();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberSubsystem.leftWinch.getSelectedSensorPosition() < clicksNeeded) {
      climberSubsystem.runMotors();
    }
    else {
      climberSubsystem.runBackMotors(); // this line will never run because if its greater, the conditional in the isFinished() command will already stop the command
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (clicksTravelled >= clicksNeeded) { 
      return true;
    } else {
      clicksTravelled = Math.abs(climberSubsystem.getWinchEncoders());
      return false;
    }
  }

  
}
