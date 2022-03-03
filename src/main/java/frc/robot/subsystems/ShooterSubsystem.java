// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public WPI_TalonFX rightShooter;
  public WPI_TalonFX leftShooter;
  public WPI_TalonFX accelerator;

  public double initialTargetVelocity = 2000;
  public double initialTargetVelocityLow = 3500;
  public double initialTargetVelocityHigh = 6500;
  public double sliderAddTargetVelocity;
  public double targetVelocity;
  public double targetVelocityLow;
  public double targetVelocityHigh;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsystem() {

    // Initialize three motors, two for the shooter and one for the accelerator
    leftShooter = new WPI_TalonFX(5);  
    rightShooter = new WPI_TalonFX(6);
    accelerator = new WPI_TalonFX(7);

    // Configuring the factory default for all motors
    leftShooter.configFactoryDefault();
    rightShooter.configFactoryDefault();
    accelerator.configFactoryDefault();

    // Setting brake mode to coast
    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
    accelerator.setNeutralMode(NeutralMode.Brake);

    leftShooter.setInverted(TalonFXInvertType.CounterClockwise);
    rightShooter.setInverted(TalonFXInvertType.Clockwise);
    rightShooter.follow(leftShooter);

    // If voltage is below 1% of output, then don't run the leftShooter
    leftShooter.configNeutralDeadband(.01);

    // 
    leftShooter.configNominalOutputForward(0, kTimeoutMs);
    leftShooter.configNominalOutputReverse(0, kTimeoutMs);
    leftShooter.configPeakOutputForward(1, kTimeoutMs);
    leftShooter.configPeakOutputReverse(-1, kTimeoutMs);

    // PID gains for velocity control for shooter
    leftShooter.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
		leftShooter.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
		leftShooter.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
		leftShooter.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);

    // Further config for the accelerator motors
    // This sets the accelerator motors to 12 volts at 100%
    accelerator.enableVoltageCompensation(true);
    accelerator.configVoltageCompSaturation(12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Velocity", leftShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Velocity Setpoint", targetVelocity);
    
  }


  public void setShooterSpeed(double slider) {
    // Getting the value of the slider and adjust the velocity of the shooter accordingly
    // The slider goes from -1 to 1, and the math scales it from 0 to 1, then multiply by 5000 to get the 
    // ticks per 100ms to add to the base target velocity
    // sliderAddTargetVelocity = (((slider*-1) + 1)/2)*8000;

    // The setpoint for the shooter is equal to the base target velocity plus the velocity added by the slider
    targetVelocity = initialTargetVelocity + sliderAddTargetVelocity;
    leftShooter.set(TalonFXControlMode.Velocity, targetVelocity);

  }

  public void setJoystickShooterSpeedLowGoal(double slider) {
    sliderAddTargetVelocity = (((slider*-1) + 1)/2)*1000;
    targetVelocityLow = initialTargetVelocityLow + sliderAddTargetVelocity;

    leftShooter.set(TalonFXControlMode.Velocity, targetVelocityLow);
  }

  public void setJoystickShooterSpeedHighGoal(double slider) {
    sliderAddTargetVelocity = (((slider*-1) + 1)/2)*2000;

    targetVelocityHigh = initialTargetVelocityHigh + sliderAddTargetVelocity;
    leftShooter.set(TalonFXControlMode.Velocity, targetVelocityHigh);
  }

  public void setShooterSpeedHighGoal() {
    leftShooter.set(TalonFXControlMode.Velocity, -6500);
    targetVelocity = 6500;
  }

  public void setShooterSpeedLowGoal() {
    leftShooter.set(TalonFXControlMode.Velocity, -2250); //WAS 3500
    targetVelocity = 2500;
  }

  public void stopShooter() {
    leftShooter.set(ControlMode.PercentOutput, 0);
  }

  public void runAccelerator() {
    accelerator.set(ControlMode.PercentOutput, -1);
  }

  public void stopAccelerator() {
    accelerator.set(ControlMode.PercentOutput, 0);
  }

  public void fullShooterSpeed() {
    leftShooter.set(ControlMode.PercentOutput, 1);
  }

  // //TEST METHODS FOR SHOOTER SPEEDS: DELETE LATER!!!!
  // public void runShooterS3() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.3);
  // }

  // public void runShooterS4() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.4);
  // }

  // public void runShooterS5() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.5);
  // }

  // public void runShooterS6() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.6);
  // }

  // public void runShooterS7() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.7);
  // }

  // public void runShooterS8() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.8);
  // }

  // public void runShooterS9() {
  //   leftShooter.set(TalonFXControlMode.PercentOutput, 0.9);
  // }
  //public void runShooter() {  
  //  shooter.set(ControlMode.PercentOutput, 1);
  //}
    
}

