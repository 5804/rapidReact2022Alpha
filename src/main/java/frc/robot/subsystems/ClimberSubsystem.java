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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public WPI_TalonFX rightWinch;
  public WPI_TalonFX leftWinch;

  public Compressor compressor;
  public DoubleSolenoid topPiston;
  public DoubleSolenoid bottomPiston;
  public DoubleSolenoid hookPiston;

  public ClimberSubsystem() {
    leftWinch = new WPI_TalonFX(3);
    rightWinch = new WPI_TalonFX(4);

    leftWinch.configFactoryDefault();
    rightWinch.configFactoryDefault();

    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

    topPiston = new DoubleSolenoid(1,PneumaticsModuleType.REVPH, 2, 3);
    bottomPiston = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1, 0);
    hookPiston = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 4, 5);
    //solenoid = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 1, 0); // this is suspect, the first argument of this function wasn't originally there
    compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    compressor.enableDigital();

    resetWinchEncoders();
    activateHookPiston();
    deactivateTopPiston();
    activateBottomPiston();
    
  }

  public void activateTopPiston() {
    topPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivateTopPiston() {
    topPiston.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void activateBottomPiston() {
    bottomPiston.set(DoubleSolenoid.Value.kForward);
   }
 
   public void deactivateBottomPiston() {
    bottomPiston.set(DoubleSolenoid.Value.kReverse);
   }

  public void activateHookPiston() {
    hookPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivateHookPiston() {
    hookPiston.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Winch Value", rightWinch.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Winch Value", leftWinch.getSelectedSensorPosition());
  }

  public void runMotors() {
    leftWinch.set(ControlMode.PercentOutput, 1.0);
    rightWinch.set(ControlMode.PercentOutput, 1.0);
  }

  public void runBackMotors() {
    leftWinch.set(ControlMode.PercentOutput, -1.0);
    rightWinch.set(ControlMode.PercentOutput, -1.0);
  }

  public void runRightMotor() {
    rightWinch.set(ControlMode.PercentOutput, 0.5);

  }

  public void runRightMotorBack() {
    rightWinch.set(ControlMode.PercentOutput, -0.5);

  }

  public void runLeftMotor() {
    leftWinch.set(ControlMode.PercentOutput, 0.5);
  }

  public void runLeftMotorBack() {
    leftWinch.set(ControlMode.PercentOutput, -0.5);
  }

  public void stopMotors() {
    leftWinch.set(0);
    rightWinch.set(0);
  }

  public void resetWinchEncoders() {
    leftWinch.setSelectedSensorPosition(0);
    rightWinch.setSelectedSensorPosition(0);
  }
}
