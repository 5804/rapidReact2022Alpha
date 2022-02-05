// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToDistanceCommand extends CommandBase {
  /** Creates a new DriveToDistanceCommand. */

  DrivetrainSubsystem drivetrainSubsystem;
  public double encoderClicks;
  public double desiredDistance;
  public double initialClicks;
  public double initialClicksAverage;
  public double desiredClicks;

  public DriveToDistanceCommand(DrivetrainSubsystem dts, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    drivetrainSubsystem = dts;
    desiredDistance = distance;
    addRequirements(drivetrainSubsystem);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
      tab.addNumber("Left Module Inches", ()->getClicksNeeded(distance));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderClicks = getClicksNeeded(desiredDistance);
    initialClicks = drivetrainSubsystem.getAverageEncoderValues();
    desiredClicks = initialClicks + encoderClicks;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.driveForward(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // this if statement checks the reading of the encoder of the front left drive motor
    if (drivetrainSubsystem.getAverageEncoderValues() >= desiredClicks) { 
      return true;
    } else {
      return false;

    }
  }

  public double getClicksNeeded(double desiredDistance) {
    // this is a function to convert desired distance in meters to the number of clicks
    // double clicksPerDegree = (2048/360);
    // double clicksPerRadians = Units.degreesToRadians(clicksPerDegree);
    // double wheelRadius = Units.inchesToMeters(2); 
    // double clicks = (desiredDistance/wheelRadius)*clicksPerRadians;
    double tickin = ((2048*6.86)/Math.PI*2*0.051);
    double clicks = tickin*desiredDistance;
    return clicks;  
}
}
