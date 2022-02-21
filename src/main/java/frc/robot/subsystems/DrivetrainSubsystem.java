// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

      public ProfiledPIDController thetaController =
      new ProfiledPIDController(
          kPThetaController, 0, 0, kThetaControllerConstraints);


  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 10.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
          SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;

//public static final double MAX_VELOCITY_METERS_PER_SECOND = 20.0;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
        private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
Pose2d targetPose;

public double target = (getGyroscopeRotation().getDegrees());

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Needed for swerve drive
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.FAST,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).addNumber("Back Left Module", ()->m_backLeftModule.getDriveEncoderValue());
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).addNumber("Back Right Module", ()->m_backRightModule.getDriveEncoderValue());
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).addNumber("Front Left Module", ()->m_frontLeftModule.getDriveEncoderValue());
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).addNumber("Front Right Module", ()->m_frontRightModule.getDriveEncoderValue());
      
      
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
   // m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }

  public double getRawRoation() {
      return m_navx.getRotation2d().getDegrees();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
 //   return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

 //    FIXME Uncomment if you are using a NavX
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
         return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  // Need for autonomous command
  public Pose2d getPose() {
        return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, getGyroscopeRotation());
  }

  public void resetGyro() {
        m_navx.zeroYaw();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);

        m_frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[0].angle.getRadians());
        m_frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[1].angle.getRadians());
        m_backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
        m_backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());

        desiredStates[0].speedMetersPerSecond = Math.abs(m_frontLeftModule.getDriveVelocity());
        desiredStates[1].speedMetersPerSecond = Math.abs(m_frontRightModule.getDriveVelocity());
        desiredStates[2].speedMetersPerSecond = Math.abs(m_backLeftModule.getDriveVelocity());
        desiredStates[3].speedMetersPerSecond = Math.abs(m_backRightModule.getDriveVelocity());
        m_odometry.update(getGyroscopeRotation(), desiredStates);

        SmartDashboard.putNumber("Current X", getPose().getX()); 
        SmartDashboard.putNumber("Current Y", getPose().getY()); 
        SmartDashboard.putNumber("Auto Angle", getPose().getRotation().getDegrees()); 
  }

  @Override
  public void periodic() {
    
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    SmartDashboard.putNumber("Raw Angle", getRawRoation());
    SmartDashboard.putNumber("current angle", getGyroscopeRotation().getDegrees());
    SmartDashboard.putNumber("Current X", getPose().getX()); 
    SmartDashboard.putNumber("Current Y", getPose().getY());

    
//     SmartDashboard.putNumber("Target", target);

      
      SmartDashboard.putNumber("Current Angle", getPose().getRotation().getDegrees()); 
     // SmartDashboard.putNumber("Target Pose Angle", targetPose.getRotation().getDegrees());






  }


  public double getClicksNeeded(double desiredDistance) {
        double clicksPerDegree = ((2048*6.86)/360);
        double clicksPerRadians = Units.degreesToRadians(clicksPerDegree);
        double wheelRadius = Units.inchesToMeters(2); 
        double clicks = (desiredDistance/wheelRadius)*clicksPerRadians;
        return clicks;  
  }

  public void driveForward(double speed) {
        m_frontLeftModule.set(speed, 0);
        m_frontRightModule.set(speed, 0);
        m_backLeftModule.set(speed, 0);
        m_backRightModule.set(speed, 0);
  }

  public void driveBackward(double speed) {
      m_frontLeftModule.set(-1*speed, 0);
      m_frontRightModule.set(-1*speed, 0);
      m_backLeftModule.set(-1*speed, 0);
      m_backRightModule.set(-1*speed, 0);
  }

  public void rotate(double rotationSpeed) {
        drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                0, // translation x supplier is 0
                0, // translation y supplier is 0
                rotationSpeed, // we only want the robot to rotate, so this value is nonzero
                new Rotation2d(0) // i dont know why we need this line...
                )
        );
  }

  public double getFrontLeftEncoderValue() {
        return m_frontLeftModule.getDriveEncoderValue();
  }
  public double getFrontRightEncoderValue() {
        return m_frontRightModule.getDriveEncoderValue();
  }
  public double getBackRightEncoderValue() {
        return m_backRightModule.getDriveEncoderValue();
  }
  public double getBackLeftEncoderValue() {
        return m_backLeftModule.getDriveEncoderValue();
  }

  public void resetEncoders() {
        m_frontLeftModule.resetDriveEncoder();
        m_frontRightModule.resetDriveEncoder();
        m_backLeftModule.resetDriveEncoder();
        m_backRightModule.resetDriveEncoder();
  }
  

}
