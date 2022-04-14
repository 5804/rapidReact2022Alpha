// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double clicksTravelled = 0;

  public DriveToDistanceCommand(DrivetrainSubsystem dts, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    drivetrainSubsystem = dts;
    desiredDistance = distance;
    addRequirements(drivetrainSubsystem);
    // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    //   tab.addNumber("Left Module Clicks", ()->getClicksNeeded(distance));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clicksTravelled = 0;
    drivetrainSubsystem.resetEncoders();
    desiredClicks = getClicksNeeded(desiredDistance);
    while (drivetrainSubsystem.getFrontRightEncoderValue() != 0) {
      drivetrainSubsystem.resetEncoders();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  //  SmartDashboard.putNumber("Desired Clicks", desiredClicks);
  
   if (desiredDistance > 0) {
    drivetrainSubsystem.driveForward(3);
   }
   else {
     drivetrainSubsystem.driveBackward(3);
   }
   
  // drivetrainSubsystem.drive(new ChassisSpeeds(0.2, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    drivetrainSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // this if statement checks the reading of the encoder used to check average encoder values. 
    // we will now make it read the encoder values of only one encoder, after running reset encoder value method
    if (clicksTravelled >= desiredClicks) { 
      // SmartDashboard.putBoolean("Is Finished", true);
      return true;
    } else {
      // SmartDashboard.putBoolean("Is Finished", false);
      clicksTravelled = Math.abs(drivetrainSubsystem.getFrontRightEncoderValue());
      return false;
    }
  }

  public double getClicksNeeded(double desiredDistance) {
    // this is a function to convert desired distance in meters to the number of clicks
    // double clicksPerDegree = (2048/360);
    // double clicksPerRadians = Units.degreesToRadians(clicksPerDegree);
    // double wheelRadius = Units.inchesToMeters(2); 
    // double clicks = (desiredDistance/wheelRadius)*clicksPerRadians;

    // this will deliver ticks needed given a desired distance in inches
    double tickin = ((2048*6.86)/(Math.PI*4));
    double clicks = tickin*desiredDistance;
    clicks = Math.abs(clicks);
    return clicks;  
  }
}
