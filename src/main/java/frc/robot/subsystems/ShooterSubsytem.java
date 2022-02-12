// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {

  public WPI_TalonFX shooter;
  public WPI_TalonFX shooterPH1;
  public WPI_TalonFX shooterPH2;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    shooter = new WPI_TalonFX(60);  
    shooterPH1 = new WPI_TalonFX(61); 
    shooterPH2 = new WPI_TalonFX(62);


    //FIXME PH=Placeholder (CHANGE THIS!!)
    shooter.configFactoryDefault();
    shooter.setNeutralMode(NeutralMode.Coast);

    shooterPH1.configFactoryDefault();
    shooterPH1.setNeutralMode(NeutralMode.Coast);

    shooterPH2.configFactoryDefault();
    shooterPH2.setNeutralMode(NeutralMode.Coast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter() {  
    shooter.set(ControlMode.PercentOutput, 1.0);
  }

  public void runShooterPH1 () {
    shooterPH1.set(ControlMode.PercentOutput, 1.0);
  }

  public void runShooterPH2 () {
    shooterPH2.set(ControlMode.PercentOutput, 1.0);
  }
  
  public void stopShooter() {
    shooter.set(ControlMode.PercentOutput, 0);
  }

  public void stopShooterPH1() {
    shooterPH1.set(ControlMode.PercentOutput, 0);
  }

  public void stopShooterPH2() {
    shooterPH2.set(ControlMode.PercentOutput, 0);
  }
    
}

