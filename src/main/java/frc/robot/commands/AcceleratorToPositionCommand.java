// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AcceleratorToPositionCommand extends CommandBase {
  /** Creates a new ConveyorToPositionCommand. */

  private final ShooterSubsystem shooterSubsystem;
  
  public double encoderClicks;
  public double desiredDistance;
  public double initialClicks;
  public double initialClicksAverage;
  public double desiredClicks;
  private double clicksTravelled = 0;
  private int directionToRotate = 0;

  // Distance is the number of clicks
  // One rotation is 2048 ticks
  public AcceleratorToPositionCommand(ShooterSubsystem ss, double distance, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.

    shooterSubsystem = ss;
    desiredDistance = distance;
    directionToRotate = direction;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clicksTravelled = 0;

    desiredClicks = desiredDistance;
    // while (shooterSubsystem.getAcceleratorEncoderValue() != 0) {
      shooterSubsystem.resetAccEncoder();
    // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (directionToRotate > 0) {
      shooterSubsystem.runAccelerator();
    } else if (directionToRotate <= 0) {
      shooterSubsystem.runAcceleratorReverse();
    } 
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAccelerator();
    shooterSubsystem.resetAccEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (clicksTravelled >= desiredClicks) { 
      // SmartDashboard.putBoolean("Is Finished (Conveyor)", true);
      return true;
    } else {
      // SmartDashboard.putBoolean("Is Finished (Conveyor)", false);
      clicksTravelled = Math.abs(shooterSubsystem.getAcceleratorEncoderValue());
      return false;
    }
  }
}
