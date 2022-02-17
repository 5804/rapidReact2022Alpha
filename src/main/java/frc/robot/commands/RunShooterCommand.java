// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsytem;

public class RunShooterCommand extends CommandBase {
  private final ShooterSubsytem shooterSubsytem;
  private final Joystick leftStick;

  /** Creates a new RunShooter. */
  public RunShooterCommand(ShooterSubsytem shooter, Joystick left) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsytem = shooter;
    leftStick = left;

    addRequirements(shooterSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsytem.setShooterSpeed(leftStick.getRawAxis(3));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsytem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
