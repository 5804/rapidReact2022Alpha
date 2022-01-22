// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestPistonSubsystem extends SubsystemBase {
  /** Creates a new TestPistonSubsystem. */

  public Compressor compressor;
  public DoubleSolenoid solenoid;

  public TestPistonSubsystem() {
    solenoid = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 1, 0);
    // solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0); // this is suspect, the first argument of this function wasn't originally there
    compressor = new Compressor(50, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void activatePiston() {
   solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void deactivatePiston() {
 solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
