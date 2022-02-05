// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public WPI_TalonFX rightWinch;
  public WPI_TalonFX leftWinch;

  public Compressor compressor;
  public DoubleSolenoid solenoid;
  public DoubleSolenoid solenoid2;
  public DoubleSolenoid solenoid3;

  public ClimberSubsystem() {
    leftWinch = new WPI_TalonFX(2);
    rightWinch = new WPI_TalonFX(1);

    leftWinch.configFactoryDefault();
    rightWinch.configFactoryDefault();

    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

    solenoid = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 1, 0);
    solenoid2 = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 2, 3);
    solenoid3 = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 4, 5);
    //solenoid = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 1, 0); // this is suspect, the first argument of this function wasn't originally there
    compressor = new Compressor(50, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();
  }

  public void activatePiston() {
   solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivatePiston() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void activatePiston2() {
    solenoid2.set(DoubleSolenoid.Value.kForward);
   }
 
   public void deactivatePiston2() {
    solenoid2.set(DoubleSolenoid.Value.kReverse);
   }

  public void activatePiston3() {
    solenoid3.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivatePiston3() {
    solenoid3.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors() {
    leftWinch.set(ControlMode.PercentOutput, 1.0);
    rightWinch.set(ControlMode.PercentOutput, 1.0);
  }

  public void runBackMotors() {
    leftWinch.set(ControlMode.PercentOutput, -1.0);
    rightWinch.set(ControlMode.PercentOutput, -1.0);
  }

  public void stopMotors() {
    leftWinch.set(0);
    rightWinch.set(0);
  }
}
