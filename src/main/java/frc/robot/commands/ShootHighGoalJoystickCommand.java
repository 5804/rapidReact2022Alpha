// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootHighGoalJoystickCommand extends CommandBase {
  /** Creates a new ShootHighGoalCommand. */

  private final ShooterSubsystem shooterSubsystem;
  private final Joystick joystick;

  public ShootHighGoalJoystickCommand(ShooterSubsystem shoot, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = shoot;
    joystick = stick;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setJoystickShooterSpeedHighGoal(joystick.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
