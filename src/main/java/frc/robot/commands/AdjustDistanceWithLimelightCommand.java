// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;

// public class AdjustDistanceWithLimelightCommand extends CommandBase {

//   private final LimelightSubsystem limelightSubsystem;
//   private final DrivetrainSubsystem drivetrainSubsystem;
//   private double idealty; // this is a value we need to find through trial and error
//   private double currentty;

//   /** Creates a new AdjustDistanceWithLimelightCommand. */
//   public AdjustDistanceWithLimelightCommand(LimelightSubsystem ls, DrivetrainSubsystem dts) {
//     // Use addRequirements() here to declare subsystem dependencies.

//     limelightSubsystem = ls;
//     drivetrainSubsystem = dts;

//     addRequirements(limelightSubsystem, drivetrainSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     currentty = limelightSubsystem.getDrivingValue();

//     // there is an ideal distance we should be away from the target and the distance we are currently away from the target
//     // if ty is too big, drive backward. else, drive forward. (there should probably be some deadband)

//     if (currentty > idealty) {
//       drivetrainSubsystem.drive(
//       ChassisSpeeds.fromFieldRelativeSpeeds(
//         -1, // translation x supplier is 0
//         -1, // translation y supplier is 0
//         0, // we only want the robot to rotate, so this value is nonzero
//         new Rotation2d(0) // i dont know why we need this line...
//       )
//     );
//     } else if (currentty < idealty) {
//       drivetrainSubsystem.drive(
//       ChassisSpeeds.fromFieldRelativeSpeeds(
//         1, // translation x supplier is 0
//         1, // translation y supplier is 0
//         0, // we only want the robot to rotate, so this value is nonzero
//         new Rotation2d(0) // i dont know why we need this line...
//       )
//     );
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivetrainSubsystem.drive(
//       ChassisSpeeds.fromFieldRelativeSpeeds(
//         0, // translation x supplier is 0
//         0, // translation y supplier is 0
//         0, // we only want the robot to rotate, so this value is nonzero
//         new Rotation2d(0) // i dont know why we need this line...
//       )
//     );
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if (currentty == idealty) {
//       return true;
//     } else {
//       return false;
//     }
//   }
// }
