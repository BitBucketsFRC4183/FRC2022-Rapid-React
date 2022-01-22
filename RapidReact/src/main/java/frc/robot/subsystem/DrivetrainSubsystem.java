// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.utils.modifiedswervelib.BitBucketsMk4SwerveModuleHelper;
import java.util.List;

public class DrivetrainSubsystem extends BitBucketsSubsystem {

  // Measure the drivetrain's maximum velocity or calculate the theoretical.
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
  public double maxVelocity_metersPerSecond;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  public double maxAngularVelocity_radiansPerSecond;

  //Instance Variables
  private SwerveDriveKinematics kinematics;

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //  Remove if you are using a Pigeon
  //  Uncomment if you are using a NavX
  private AHRS navX;

  //Swerve Modules
  private SwerveModule moduleFrontLeft;
  private SwerveModule moduleFrontRight;
  private SwerveModule moduleBackLeft;
  private SwerveModule moduleBackRight;

  private Translation2d moduleFrontLeftLocation;
  private Translation2d moduleFrontRightLocation;
  private Translation2d moduleBackLeftLocation;
  private Translation2d moduleBackRightLocation;

  private ChassisSpeeds chassisSpeeds;

  private Pose2d pose;
  private SwerveDriveOdometry odometry;

  private Field2d field;
  private Trajectory trajectory;

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

    this.navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    odometry =
      new SwerveDriveOdometry(
        kinematics,
        getGyroscopeRotation(), // this was "getgyroheading"
        new Pose2d(0, 0, new Rotation2d(0))
      );

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    trajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(5, 3), new Translation2d(10, 3)),
        new Pose2d(10, 5, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
      );

    this.field = new Field2d();
    SmartDashboard.putData(field);
    this.field.getObject("traj").setTrajectory(trajectory);

    this.initializeModules();
  }

  private void initializeModules() {
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
    // Setup motor configuration
    moduleFrontLeft =
      BitBucketsMk4SwerveModuleHelper.createWPI_TalonFX(
        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        BitBucketsMk4SwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor
        config.frontLeftModuleDriveMotor,
        // This is the ID of the steer motor
        config.frontLeftModuleSteerMotor,
        // This is the ID of the steer encoder
        config.frontLeftModuleSteerEncoder,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        config.drive.frontLeftModuleSteerOffset
      );

    // We will do the same for the other modules
    moduleFrontRight =
      BitBucketsMk4SwerveModuleHelper.createWPI_TalonFX(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
        BitBucketsMk4SwerveModuleHelper.GearRatio.L2,
        config.frontRightModuleDriveMotor,
        config.frontRightModuleSteerMotor,
        config.frontRightModuleSteerEncoder,
        config.drive.frontRightModuleSteerOffset
      );

    moduleBackLeft =
      BitBucketsMk4SwerveModuleHelper.createWPI_TalonFX(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
        BitBucketsMk4SwerveModuleHelper.GearRatio.L2,
        config.backLeftModuleDriveMotor,
        config.backLeftModuleSteerMotor,
        config.backLeftModuleSteerEncoder,
        config.drive.backLeftModuleSteerOffset
      );

    moduleBackRight =
      BitBucketsMk4SwerveModuleHelper.createWPI_TalonFX(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
        BitBucketsMk4SwerveModuleHelper.GearRatio.L2,
        config.backRightModuleDriveMotor,
        config.backRightModuleSteerMotor,
        config.backRightModuleSteerEncoder,
        config.drive.backRightModuleSteerOffset
      );
  }

  public DrivetrainSubsystem(Config config) {
    super(config);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // Remove if you are using a Pigeon
    //m_pigeon.setFusedHeading(0.0);

    // Uncomment if you are using a NavX
    this.navX.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // Remove if you are using a Pigeon
    //  return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // Uncomment if you are using a NavX
    if (this.navX.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(this.navX.getFusedHeading());
    }

    //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - this.navX.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity_metersPerSecond);

    moduleFrontLeft.set(
      states[0].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage,
      states[0].angle.getRadians()
    );
    moduleFrontRight.set(
      states[1].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage,
      states[1].angle.getRadians()
    );
    moduleBackLeft.set(
      states[2].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage,
      states[2].angle.getRadians()
    );
    moduleBackRight.set(
      states[3].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage,
      states[3].angle.getRadians()
    );

    var gyroAngle = Rotation2d.fromDegrees(-navX.getAngle());
    pose = odometry.update(gyroAngle, states[0], states[1], states[2], states[3]);

    field.setRobotPose(odometry.getPoseMeters());
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public void disable() {
    stop();
  }
}
