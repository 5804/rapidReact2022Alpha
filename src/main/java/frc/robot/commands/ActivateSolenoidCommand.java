// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestPistonSubsystem;

public class ActivateSolenoidCommand extends CommandBase {
  /** Creates a new ActivateSolenoidCommand. */

  private final TestPistonSubsystem testPistonSubsystem;

  public ActivateSolenoidCommand(TestPistonSubsystem tps) {
    // Use addRequirements() here to declare subsystem dependencies.

    testPistonSubsystem = tps;
    addRequirements(testPistonSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testPistonSubsystem.activatePiston();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testPistonSubsystem.deactivatePiston();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
