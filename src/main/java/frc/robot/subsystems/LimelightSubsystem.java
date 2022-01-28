/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("tx", tx);

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    SmartDashboard.putBoolean("tv", tv >= 1.0);
  }

  public double getSteeringValue() {
    double STEER_K = 0.03;  

    // how hard to turn toward the target
    // final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    // final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    // final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double signumtx = Math.signum(tx);

    if (isTargetValid() == false) {
      return 0.0;
    }

    double txAbs = Math.abs(tx);
    double txDeadband = txAbs - Constants.LIMELIGHT_DEADBAND;

    if (txDeadband < 0) {
      return 0.0;
    } 

    double minDriveWithSine = signumtx * Constants.MIN_STEER_K;
    double steer_cmd = tx * STEER_K;
    double finalSteerCmd = minDriveWithSine + steer_cmd;

    return finalSteerCmd;
  }

  public boolean isTargetValid() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    return tv >= 1.0;
  }

  public boolean isRedPath() {
    // If red path reutrn true else return false
    // This is determined by the limelights first reading
    // 
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
    // for the blue path for both paths, the area is .25%
    // for the red path for both paths, the area is 2.0%

    // ta for red: 2.1%
    // ta for blue: 

    if (ta > 1.0) {
      return true;
      // and drive forward 5 feed
    } else {
      return false;
      // and drive forward 12 feed
    }
  }

  public boolean isRedPathA() {
    // IF redpath A return true else return false
    // Determined by the limelights second reading

    // but how do we determine if a ball is right in front of us
    // so we have to put the robot in the place that it will be after picking up the first ball and then look at the ta values

    // if ta < .01, then path A, else path B

    // ta for red path A: 
    // ta for red path B:

    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (ta < 0.2) {
      return true;
    } else {
      return false;
    }

  }

  public boolean isBluePathA() {
    // IF redpath A return true else return false
    // Determined by the limelights second reading

    // ta for blue path A: 
    // ta for blue path B: 

    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (ta < 0.2) {
      return true;
    } else {
      return false;
    }
  }

  // public boolean isAligned() {
  //   // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

  //   if (isTargetValid() == false) {
  //     return true;
  //   }

  //   // tx = Math.abs(tx);
  //   // tx = tx - Constants.LIMELIGHT_DEADBAND;

  //   // if (tx < 0) {
  //   //   return true;
  //   // } 

  //   return false;
  // }
}
