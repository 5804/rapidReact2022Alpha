// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ConveyorToPositionCommand extends CommandBase {
  /** Creates a new ConveyorToPositionCommand. */

  private final IntakeSubsystem intakeSubsystem;
  
  public double encoderClicks;
  public double desiredDistance;
  public double initialClicks;
  public double initialClicksAverage;
  public double desiredClicks;
  private double clicksTravelled = 0;
  private int directionToRotate = 0;

  // Distance is the number of clicks
  // One rotation is 2048 ticks
  public ConveyorToPositionCommand(IntakeSubsystem is, double distance, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.

    intakeSubsystem = is;
    desiredDistance = distance;
    directionToRotate = direction;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clicksTravelled = 0;

    desiredClicks = desiredDistance;
    // while (intakeSubsystem.getConveyorEncoderValue() != 0) {
      intakeSubsystem.resetConveyorEncoder();
    // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (directionToRotate > 0) {
      intakeSubsystem.runConveyorMotor();
    } else if (directionToRotate <= 0) {
      intakeSubsystem.runConveyorMotorBackward();
    } 
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopConveyorMotor();
    intakeSubsystem.resetConveyorEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (clicksTravelled >= desiredClicks) { 
      // SmartDashboard.putBoolean("Is Finished (Conveyor)", true);
      return true;
    } else {
      // SmartDashboard.putBoolean("Is Finished (Conveyor)", false);
      clicksTravelled = Math.abs(intakeSubsystem.getConveyorEncoderValue());
      return false;
    }
  }
}
