// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.swervelib.Gyroscope;
import frc.swervelib.GyroscopeHelper;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.PoseTelemetry;
import frc.swervelib.SdsModuleConfigurations;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveModule;
import java.util.ArrayList;

public class DrivetrainSubsystem extends BitBucketsSubsystem {

  // Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public double maxVelocity_metersPerSecond;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  public double maxAngularVelocity_radiansPerSecond;

  // Instance Variables
  private SwerveDriveKinematics kinematics;

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //  Remove if you are using a Pigeon
  //  Uncomment if you are using a NavX
  private Gyroscope gyro;

  // Swerve Modules
  private SwerveModule moduleFrontLeft;
  private SwerveModule moduleFrontRight;
  private SwerveModule moduleBackLeft;
  private SwerveModule moduleBackRight;
  private ArrayList<SwerveModule> modules;

  private Translation2d moduleFrontLeftLocation;
  private Translation2d moduleFrontRightLocation;
  private Translation2d moduleBackLeftLocation;
  private Translation2d moduleBackRightLocation;

  private ChassisSpeeds chassisSpeeds;

  private Pose2d pose;
  private SwerveDriveOdometry odometry;

  public Field2d field;

  SwerveDrivetrainModel dt;

  @Override
  public void init() {
    this.maxVelocity_metersPerSecond =
      6380.0 /
      60.0 *
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() *
      Math.PI;

    this.maxAngularVelocity_radiansPerSecond =
      maxVelocity_metersPerSecond /
      Math.hypot(config.drive.drivetrainTrackWidth_meters / 2.0, config.drive.drivetrainWheelBase_meters / 2.0);

    this.moduleFrontLeftLocation =
      new Translation2d(config.drive.drivetrainTrackWidth_meters / 2.0, config.drive.drivetrainWheelBase_meters / 2.0);
    this.moduleFrontRightLocation =
      new Translation2d(config.drive.drivetrainTrackWidth_meters / 2.0, -config.drive.drivetrainWheelBase_meters / 2.0);
    this.moduleBackLeftLocation =
      new Translation2d(-config.drive.drivetrainTrackWidth_meters / 2.0, config.drive.drivetrainWheelBase_meters / 2.0);
    this.moduleBackRightLocation =
      new Translation2d(
        -config.drive.drivetrainTrackWidth_meters / 2.0,
        -config.drive.drivetrainWheelBase_meters / 2.0
      );

    this.kinematics =
      new SwerveDriveKinematics(
        moduleFrontLeftLocation,
        moduleFrontRightLocation,
        moduleBackLeftLocation,
        moduleBackRightLocation
      );

    // this.navX = new AHRS(SPI.Port.kMXP, (byte) 200)
    // navXFactoryBuilder n = new navXFactoryBuilder();
    this.gyro = GyroscopeHelper.createnavXMXP();

    setOdometry(config.drive.defaultStartingPosition);

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    // List.of(new Translation2d(5, 3), new Translation2d(10, 3)),
    // new Pose2d(10, 5, Rotation2d.fromDegrees(0)),
    // new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
    // );

    SmartDashboard.putData(field);

    this.initializeModules();
  }

  private void initializeModules() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use
    // a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // Setup motor configuration
    moduleFrontLeft =
      Mk4SwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4SwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor
        config.frontLeftModuleDriveMotor,
        // This is the ID of the steer motor
        config.frontLeftModuleSteerMotor,
        // This is the ID of the steer encoder
        config.frontLeftModuleSteerEncoder,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        config.drive.frontLeftModuleSteerOffset,
        "FL"
      );

    // We will do the same for the other modules
    moduleFrontRight =
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        config.frontRightModuleDriveMotor,
        config.frontRightModuleSteerMotor,
        config.frontRightModuleSteerEncoder,
        config.drive.frontRightModuleSteerOffset,
        "FR"
      );

    moduleBackLeft =
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        config.backLeftModuleDriveMotor,
        config.backLeftModuleSteerMotor,
        config.backLeftModuleSteerEncoder,
        config.drive.backLeftModuleSteerOffset,
        "BL"
      );

    moduleBackRight =
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        config.backRightModuleDriveMotor,
        config.backRightModuleSteerMotor,
        config.backRightModuleSteerEncoder,
        config.drive.backRightModuleSteerOffset,
        "BR"
      );

    // We will also create a list of all the modules so we can easily access them later
    modules =
      new ArrayList<SwerveModule>() {
        {
          add(moduleFrontLeft);
          add(moduleFrontRight);
          add(moduleBackLeft);
          add(moduleBackRight);
        }
      };
    dt = new SwerveDrivetrainModel(modules, gyro, this.kinematics);
  }
  
  public void zeroGyroscope()
  {
    gyro.zeroGyroscope();
  }
  
  public Rotation2d getGyroscopeRotation() {
    return gyro.getGyroHeading();
  }

  public DrivetrainSubsystem(Config config) {
    super(config);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    dt.setModuleStates(chassisSpeeds);
    SwerveModuleState[] states = dt.getSwerveModuleStates();
    // SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(chassisSpeeds);
    
    if (states != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity_metersPerSecond);
      
      modules.get(0).set(states[0].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage, states[0].angle.getRadians());
      modules.get(1).set(states[1].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage, states[1].angle.getRadians());
      modules.get(2).set(states[2].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage, states[2].angle.getRadians());
      modules.get(3).set(states[3].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage, states[3].angle.getRadians());
    }

    var gyroAngle = gyro.getGyroHeading().times(-1);
    // var gyroAngle = Rotation2d.fromDegrees(-navX.getAngle());
    pose = odometry.update(gyroAngle, states[0], states[1], states[2], states[3]);
    
    field.setRobotPose(odometry.getPoseMeters());
    
    dt.updateTelemetry();
  }

  public void setOdometry(Pose2d startingPosition) {
    odometry =
    new SwerveDriveOdometry(
        kinematics,
        gyro.getGyroHeading(),
        startingPosition
        );
  }
  
  public void stop() {
    this.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
  
  
  @Override
  public void simulationPeriodic()
  {
    // logger().logNum(LogLevel.GENERAL, "aasdsad/asdasd", chassisSpeeds.omegaRadiansPerSecond);
    dt.update(DriverStation.isDisabled(), 13.2); //this.config.maxVoltage);
  }
  
  @Override
  public void disable() {
    stop();
  }
  
}
