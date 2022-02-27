// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  public WPI_TalonFX intakeMotor;
  public WPI_TalonFX conveyorMotor;

  public DoubleSolenoid intakePiston;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonFX(8);
    conveyorMotor = new WPI_TalonFX(9);
    
    intakePiston = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 6, 7);

    intakeMotor.configFactoryDefault();
    conveyorMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    conveyorMotor.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void activateIntakePiston() {
    intakePiston.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivateIntakePiston() {
    intakePiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntakeMotor() {
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.5);
  }

  public void runConveyorMotor() {
    conveyorMotor.set(TalonFXControlMode.PercentOutput, 0.5);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void stopConveyorMotor() {
    conveyorMotor.set(TalonFXControlMode.PercentOutput, 0);
  } 

}
