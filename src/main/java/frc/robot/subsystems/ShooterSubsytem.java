// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {

  public WPI_TalonFX shooter;
 // public WPI_TalonFX shooterPH1;
 // public WPI_TalonFX shooterPH2;

 // public double shooterSpeed;
  //public Joystick testJoystick = new Joystick(2);

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    shooter = new WPI_TalonFX(60);  
    //shooterPH1 = new WPI_TalonFX(61); 
    //shooterPH2 = new WPI_TalonFX(62);


    //FIXME PH=Placeholder (CHANGE THIS!!)
    shooter.configFactoryDefault();
    shooter.setNeutralMode(NeutralMode.Coast);

    //shooterPH1.configFactoryDefault();
    //shooterPH1.setNeutralMode(NeutralMode.Coast);

    //shooterPH2.configFactoryDefault();
    //shooterPH2.setNeutralMode(NeutralMode.Coast);

    //shooterSpeed = (((testJoystick.getRawAxis(3)*-1)+1)/2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Velocity", shooter.getSelectedSensorVelocity());
    
  }


  public void runShooter(double slider) {
    // Getting the value of the slider and adjust the velocity of the shooter accordingly
    // The slider goes from -1 to 1, and the math scales it from 0 to 1, then multiply by 5000 to get the 
    // ticks per 100ms to add to the base target velocity
    double sliderVelocity = (((slider*-1) + 1)/2);
    SmartDashboard.putNumber("Shooter Speed", sliderVelocity);

    shooter.set(TalonFXControlMode.PercentOutput, sliderVelocity);
    
  }

  //TEST METHODS FOR SHOOTER SPEEDS: DELETE LATER!!!!
  public void runShooterS3() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.3);
  }

  public void runShooterS4() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.4);
  }

  public void runShooterS5() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.5);
  }

  public void runShooterS6() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.6);
  }

  public void runShooterS7() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.7);
  }

  public void runShooterS8() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.8);
  }

  public void runShooterS9() {
    shooter.set(TalonFXControlMode.PercentOutput, 0.9);
  }
  //public void runShooter() {  
  //  shooter.set(ControlMode.PercentOutput, 1);
  //}

  //public void runShooterPH1 () {
  //  shooterPH1.set(ControlMode.PercentOutput, 1.0);
  //}

  //public void runShooterPH2 () {
  //  shooterPH2.set(ControlMode.PercentOutput, 1.0);
  //}
  
  public void stopShooter() {
    shooter.set(ControlMode.PercentOutput, 0);
  }

  //public void stopShooterPH1() {
  //  shooterPH1.set(ControlMode.PercentOutput, 0);
 // }

  //public void stopShooterPH2() {
  //  shooterPH2.set(ControlMode.PercentOutput, 0);
  //
//}
    
}

