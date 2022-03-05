// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActivateAcceleratorCommand;
import frc.robot.commands.ActivateBottomPistonCommand;
import frc.robot.commands.ActivateTopPistonCommand;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.DeactivateBottomPistonCommand;
import frc.robot.commands.DeactivateHookPistonCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveAtSpeed80Command;
import frc.robot.commands.DriveToDistanceCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.WinchInCommand;
import frc.robot.commands.RunConveyorMotorCommand;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.RunIntakeMotorsCommand;
import frc.robot.commands.RunLeftMotorBackCommand;
import frc.robot.commands.RunLeftMotorCommand;
import frc.robot.commands.WinchOutCommand;
import frc.robot.commands.RunRightMotor;
import frc.robot.commands.RunRightMotorBackCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.ShootHighGoalJoystickCommand;
import frc.robot.commands.ShootLowGoalCommand;
import frc.robot.commands.ShootHighGoalCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.CommandGroups.FireShooterCommandGroup;
import frc.robot.commands.CommandGroups.S1_2BallCommandGroup;
import frc.robot.commands.CommandGroups.S1_2BallLOWCommandGroup;
import frc.robot.commands.CommandGroups.S1_3BallCommandGroup;
import frc.robot.commands.CommandGroups.S2_2BallCommandGroup;
import frc.robot.commands.CommandGroups.S2_2BallLOWCommandGroup;
import frc.robot.commands.CommandGroups.S2_3BallCommandGroup;
import frc.robot.commands.CommandGroups.S3_2BallCommandGroup;
import frc.robot.commands.CommandGroups.S3_2BallLOWCommandGroup;
import frc.robot.commands.CommandGroups.S3_3BallCommandGroup;
import frc.robot.commands.CommandGroups.TestAutoDriveCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DeactivateTopPistonCommand;
import frc.robot.commands.ActivateHookPistonCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.DeactivateHookPistonCommand;

import static frc.robot.Constants.*;


public class RobotContainer {

  private final SendableChooser sendableChooser = new SendableChooser<Command>();
  
  private final XboxController m_controller = new XboxController(0);

  //Button Board
  private final Joystick climbBoard = new Joystick(1);

  POVButton upPOV = new POVButton(m_controller, 0);
  POVButton downPOV = new POVButton(m_controller, 180);
  

  //Test Shooter Joystick
  // private final Joystick shooterStick = new Joystick(2);
  
  //FOR DRIVETRAIN:
    public static final DrivetrainSubsystem driveTrainSubsystem = new DrivetrainSubsystem();
    private final DriveToDistanceCommand driveToDistanceCommand = new DriveToDistanceCommand(driveTrainSubsystem, 8);
    private final DriveAtSpeed80Command driveAtSpeed80Command = new DriveAtSpeed80Command(driveTrainSubsystem);

  //FOR LIMELIGHT:
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final AlignToGoalWithLimelightCommand alignToGoalWithLimelightCommand = new AlignToGoalWithLimelightCommand(limelightSubsystem, driveTrainSubsystem);
        
  //FOR AUTO:
      private final Command DriveToDistanceCommand = new DriveToDistanceCommand(driveTrainSubsystem, -12);
      private final TestAutoDriveCommandGroup testAutoDriveCommand = new TestAutoDriveCommandGroup(driveTrainSubsystem);
      private final TurnToAngle turnToAngle = new TurnToAngle(45, driveTrainSubsystem);

      // PathPlannerTrajectory straightDrive1 = PathPlanner.loadPath("DriveCurve2", 8, 5);

//FOR INTAKE:
    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final RunIntakeMotorsCommand runIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeSubsystem);
    private final RunConveyorMotorCommand runConveyorMotorCommand = new RunConveyorMotorCommand(intakeSubsystem);
    public final RunIntakeAndConveyor runIntakeAndConveyor = new RunIntakeAndConveyor(intakeSubsystem);


  //FOR SHOOTER:
    public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ActivateAcceleratorCommand activateAcceleratorCommand = new ActivateAcceleratorCommand(shooterSubsystem);
    //private final RunShooterCommand runShooterCommand = new RunShooterCommand(shooterSubsytem);
    private final ShootLowGoalCommand shootLowGoalCommand = new ShootLowGoalCommand(shooterSubsystem);
    private final ShootHighGoalCommand shootHighGoalCommand = new ShootHighGoalCommand(shooterSubsystem);
    private final FireShooterCommandGroup fireShooterCommandGroup = new FireShooterCommandGroup(shooterSubsystem, intakeSubsystem);

  //FOR CLIMBER:
      private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
      private final WinchOutCommand winchOutCommand = new WinchOutCommand(climberSubsystem);
      private final WinchInCommand winchInCommand = new WinchInCommand(climberSubsystem);
      private final ActivateTopPistonCommand activateTopPistonCommand = new ActivateTopPistonCommand(climberSubsystem);
      private final DeactivateTopPistonCommand deactivateTopPistonCommand = new DeactivateTopPistonCommand(climberSubsystem);
      private final ActivateBottomPistonCommand activateBottomPistonCommand = new ActivateBottomPistonCommand(climberSubsystem);
      private final DeactivateBottomPistonCommand deactivateBottomPistonCommand = new DeactivateBottomPistonCommand(climberSubsystem);
      private final ActivateHookPistonCommand activateHookPistonCommand = new ActivateHookPistonCommand(climberSubsystem);
      private final DeactivateHookPistonCommand deactivateHookPistonCommand = new DeactivateHookPistonCommand(climberSubsystem);
      private final RunRightMotor runRightMotor = new RunRightMotor(climberSubsystem);
      private final RunRightMotorBackCommand runRightMotorBackCommand = new RunRightMotorBackCommand(climberSubsystem);
      private final RunLeftMotorCommand runLeftMotorCommand = new RunLeftMotorCommand(climberSubsystem);
      private final RunLeftMotorBackCommand runLeftMotorBackCommand = new RunLeftMotorBackCommand(climberSubsystem);
      // private final RunRightMotor runRightMotor = new RunRightMotor(climberSubsystem);


  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    driveTrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            driveTrainSubsystem,
           () -> modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
           () -> modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
           () -> modifyAxis((-1*m_controller.getRightX())) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    // shooterSubsystem.setDefaultCommand(new ShootLowGoalCommand(shooterSubsystem));



      //FIXME, probably remove this, does not work
      // SmartDashboard.putNumber("Current X", driveTrainSubsystem.getPose().getX()); 
      // SmartDashboard.putNumber("Current Y", driveTrainSubsystem.getPose().getY()); 
      // SmartDashboard.putNumber("Current Angle", driveTrainSubsystem.getPose().getRotation().getDegrees()); 

    // Configure the button bindings
    configureButtonBindings();

    // All sendable chooser options
    sendableChooser.setDefaultOption("1-2Ball", new S1_2BallCommandGroup(driveTrainSubsystem, shooterSubsystem, intakeSubsystem, limelightSubsystem));
    sendableChooser.addOption("2-2Ball", new S2_2BallCommandGroup(driveTrainSubsystem, shooterSubsystem));//ADD IN REQUIRMENTS
    sendableChooser.addOption("3-2Ball", new S3_2BallCommandGroup(driveTrainSubsystem, shooterSubsystem));
    sendableChooser.addOption("1-3Ball", new S1_3BallCommandGroup(driveTrainSubsystem, shooterSubsystem, intakeSubsystem));
    sendableChooser.addOption("2-3Ball", new S2_3BallCommandGroup(driveTrainSubsystem, shooterSubsystem));
    sendableChooser.addOption("3-3Ball", new S3_3BallCommandGroup(driveTrainSubsystem, shooterSubsystem));
    sendableChooser.addOption("1-2BallLOW", new S1_2BallLOWCommandGroup(driveTrainSubsystem, shooterSubsystem, intakeSubsystem, limelightSubsystem));
    sendableChooser.addOption("2-2BallLOW", new S2_2BallLOWCommandGroup(driveTrainSubsystem, shooterSubsystem, intakeSubsystem, limelightSubsystem));
    sendableChooser.addOption("3-2BallLOW", new S3_2BallLOWCommandGroup(driveTrainSubsystem, shooterSubsystem, intakeSubsystem, limelightSubsystem));
    SmartDashboard.putData("Auto Selector", sendableChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { 
    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton) // FIXME This button press has an error because the getBackButton function does not have any code
    //         // No requirements because we don't need to interrupt anything
    //         .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    

    //FOR DRIVETRAIN:

         new Button(m_controller::getStartButton)
        .whenPressed(driveTrainSubsystem::zeroGyroscope);
        
        new LeftTriggerPressed().whileActiveContinuous(alignToGoalWithLimelightCommand);

    // FOR AUTO:
        // new Button(m_controller::getAButton)
        //  .whenPressed(DriveToDistanceCommand);

        // new Button(m_controller::getRightBumper)
        // .whenPressed(testAutoDriveCommand);

        // new Button(m_controller::getBButton)
        //    .whenPressed(turnToAngle);


    // // // new Button(m_controller::getRightBumper)
    // // //     .whileHeld(alignToGoalWithLimelightCommand);

    //FOR CLIMBER:
 

        final JoystickButton b1 = new JoystickButton(climbBoard, 1); 
        b1.whenPressed(activateTopPistonCommand); 

        final JoystickButton b2 = new JoystickButton(climbBoard, 2); 
        b2.whenPressed(deactivateTopPistonCommand); 
        
        final JoystickButton b3 = new JoystickButton(climbBoard, 3); 
        b3.whenPressed(activateBottomPistonCommand); 

        final JoystickButton b4 = new JoystickButton(climbBoard, 4); 
        b4.whenPressed(deactivateBottomPistonCommand); 

        final JoystickButton b5 = new JoystickButton(climbBoard, 5); 
        b5.whenPressed(activateHookPistonCommand); 

        final JoystickButton b6 = new JoystickButton(climbBoard, 6); 
        b6.whenPressed(deactivateHookPistonCommand); 

        new BoardStickForward().whileActiveContinuous(winchOutCommand);

        new BoardStickBackwards().whileActiveContinuous(winchInCommand);

        final JoystickButton b7 = new JoystickButton(climbBoard, 7); //MAKE RIGHT
        b7.whileHeld(runRightMotor); 

        final JoystickButton b8 = new JoystickButton(climbBoard, 8); 
        b8.whileHeld(runRightMotorBackCommand); 

        final JoystickButton b9 = new JoystickButton(climbBoard, 9); //MAKE LEFT
        b9.whileHeld(runLeftMotorCommand); 

        final JoystickButton b10 = new JoystickButton(climbBoard, 10); 
        b10.whileHeld(runLeftMotorBackCommand); 

               //  new Button(climbController::getAButton)
        //     .whileHeld(runMotorsCommand);
      
        // new Button(climbController::getBButton)
        //   .whileHeld(runBackMotorsCommand);

        // new Button(climbController::getXButton)
        //     .whileHeld(activateTopPistonCommand);

        // new Button(climbController::getYButton)
        //      .whileHeld(deactivateTopPistonCommand);

        //  new Button(climbController::getLeftBumper)
        //      .whenPressed(deactivateBottomPistonCommand);
        
        //  new Button(climbController::getRightBumper)
        //      .whileHeld(activateBottomPistonCommand);

        // new Button(climbController::getStartButton)
        //       .whenPressed(activateHookPistonCommand);

        // new Button(climbController::getBackButton)
        //       .whenPressed(deactivateHookPistonCommand);

        // new Button(climbController::getLeftStickButton)
        //       .whenPressed(climberSubsystem::resetWinchEncoders);
    


         // if you write "{subsystem}::{function in the subsystem}" it counts as a command, so we could use it in command groups

    //FOR INTAKE:
    // new Button(m_controller::getBButton).toggleWhenPressed(new StartEndCommand(shooterSubsystem::stopShooter,
    // shooterSubsystem::setShooterSpeedLowGoal,
    // shooterSubsystem));

    new Button(m_controller::getBackButton).toggleWhenPressed(new StartEndCommand(intakeSubsystem::deactivateIntakePiston, 
    intakeSubsystem::activateIntakePiston,
    intakeSubsystem));



    //FOR SHOOTER:

    new Button(m_controller::getXButton)
      .toggleWhenActive(runIntakeAndConveyor);

    new Button(m_controller::getBButton)
      .whenPressed(shooterSubsystem::stopShooter);

    new Button(m_controller::getLeftBumper)
      .whileHeld(shootLowGoalCommand);

    new Button(m_controller::getRightBumper)
      .whileHeld(shootHighGoalCommand);
      
    // final POVButton upButton = new POVButton(m_controller, 0, 1);
    // upButton.whileHeld(driveAtSpeed80Command); 

    // new POVButton(m_controller, 0).whileHeld(driveAtSpeed80Command);

    // upPOV.whileHeld(driveAtSpeed80Command);

    // new POVButton(m_controller, 0).whileHeld(driveAtSpeed80Command);
    new Button(m_controller::getAButton)
      .whenPressed(driveToDistanceCommand);

    // new Button(m_controller::getAButton)
    //   .whenPressed(shooterSubsystem::fullShooterSpeed);

    new RightTriggerPressed().whileActiveContinuous(fireShooterCommandGroup);
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // An ExampleCommand will run in autonomous

    // // Run path following command, then stop at the end.
    // return driveTrainSubsystem.createCommandForTrajectory(examplePath).andThen(() -> driveTrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
    //return swerveControllerCommand;
    // return new InstantCommand();

    // // this is a message from the push from february 2:
    // // the robot was able to move in the x direction but not the y direction

   return (Command) sendableChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public class RightTriggerPressed extends Trigger {
    @Override
    public boolean get() {
      return m_controller.getRightTriggerAxis() > 0.5;
      // This returns whether the trigger is active
    }
  }

   public class LeftTriggerPressed extends Trigger {
      @Override
      public boolean get() {
        return m_controller.getLeftTriggerAxis() > 0.5;
        // This returns whether the trigger is active
      }
    }

    public class BoardStickForward extends Trigger {
      @Override
      public boolean get() {
        return climbBoard.getRawAxis(1) > 0.7;
        // This returns whether the trigger is active
      }
    }
    
    public class BoardStickBackwards extends Trigger {
      @Override
      public boolean get() {
        return climbBoard.getRawAxis(1) < -0.7;
        // This returns whether the trigger is active
      }
    }

  public void shooterTestBindings () {
      //  BUTTONS BELOW ARE FOR TESTING SHOOTER SPEED, DELETE LATER
      //   The number corresponds to the shooter speed (ex S3 = 0.3)

      // final JoystickButton shooterButton = new JoystickButton(shooterStick, 1);
      // shooterButton.whileHeld(runShooterCommand);


      //   final JoystickButton shooterButton2 = new JoystickButton(shooterStick, 2);
      //     shooterButton2.whileHeld(shootLowGoalCommand);

      //     final JoystickButton shooterButton3 = new JoystickButton(shooterStick, 3);
      //     shooterButton3.whileHeld(shootHighGoalCommand);

      //   final JoystickButton shooterButton4 = new JoystickButton(shooterStick, 8);
      //     shooterButton4.whileHeld(shooterSubsytem::runShooterS4);

      //   final JoystickButton shooterButton5 = new JoystickButton(shooterStick, 7);
      //     shooterButton5.whileHeld(shooterSubsytem::runShooterS5);

      //   final JoystickButton shooterButton6 = new JoystickButton(shooterStick, 6);
      //     shooterButton6.whileHeld(shooterSubsytem::runShooterS6);

      //   final JoystickButton shooterButton7 = new JoystickButton(shooterStick, 9);
      //     shooterButton7.whileHeld(shooterSubsytem::runShooterS7);

      //   final JoystickButton shooterButton8 = new JoystickButton(shooterStick, 10);
      //     shooterButton8.whileHeld(shooterSubsytem::runShooterS8);

      //   final JoystickButton shooterButton9 = new JoystickButton(shooterStick, 11);
      //     shooterButton9.whileHeld(shooterSubsytem::runShooterS9);
  }


  

  
}
