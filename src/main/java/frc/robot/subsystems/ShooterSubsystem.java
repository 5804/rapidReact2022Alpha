// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public WPI_TalonFX rightShooter;
  public WPI_TalonFX leftShooter;

  public ShooterSubsystem() {
    leftShooter = new WPI_TalonFX(2);
    rightShooter = new WPI_TalonFX(1);

    leftShooter.configFactoryDefault();
    rightShooter.configFactoryDefault();

    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors() {
    leftShooter.set(ControlMode.PercentOutput, 1.0);
    rightShooter.set(ControlMode.PercentOutput, 1.0);
  }

  public void runBackMotors() {
    leftShooter.set(ControlMode.PercentOutput, -1.0);
    rightShooter.set(ControlMode.PercentOutput, -1.0);
  }

  public void stopMotors() {
    leftShooter.set(0);
    rightShooter.set(0);
  }
}
