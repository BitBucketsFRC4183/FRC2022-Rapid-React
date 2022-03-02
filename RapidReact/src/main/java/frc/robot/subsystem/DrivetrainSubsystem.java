// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.config.Config;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;

import java.util.ArrayList;
import java.util.List;
import java.util.StringJoiner;

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
  public SwerveDriveKinematics kinematics;

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //  Remove if you are using a Pigeon
  //  Uncomment if you are using a NavX
  public AHRS gyro;

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
  public SwerveDriveOdometry odometry;

  public Field2d field;

  private final Loggable<String> odometryLoggable = BucketLog.loggable(Put.STRING, "drivetrain/odometry");

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

    this.gyro = new AHRS(SPI.Port.kMXP, (byte)200);

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.initializeModules();

    setOdometry(new Pose2d());
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
    this.moduleFrontLeft = Mk4SwerveModuleHelper.createFalcon500(
            //Smart Dashboard Tab
            tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, //Gear Ratio
            config.frontLeftModuleDriveMotor_ID, //Drive Motor
            config.frontLeftModuleSteerMotor_ID, //Steer Motor
            config.frontLeftModuleSteerEncoder_ID, //Steer Encoder
            config.drive.frontLeftModuleSteerOffset //Steer Offset
    );

    this.moduleFrontRight = Mk4SwerveModuleHelper.createFalcon500(
            //Smart Dashboard Tab
            tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, //Gear Ratio
            config.frontRightModuleDriveMotor_ID, //Drive Motor
            config.frontRightModuleSteerMotor_ID, //Steer Motor
            config.frontRightModuleSteerEncoder_ID, //Steer Encoder
            config.drive.frontRightModuleSteerOffset //Steer Offset
    );

    this.moduleBackLeft = Mk4SwerveModuleHelper.createFalcon500(
            //Smart Dashboard Tab
            tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, //Gear Ratio
            config.backLeftModuleDriveMotor_ID, //Drive Motor
            config.backLeftModuleSteerMotor_ID, //Steer Motor
            config.backLeftModuleSteerEncoder_ID, //Steer Encoder
            config.drive.backLeftModuleSteerOffset //Steer Offset
    );

    this.moduleBackRight = Mk4SwerveModuleHelper.createFalcon500(
            //Smart Dashboard Tab
            tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, //Gear Ratio
            config.backRightModuleDriveMotor_ID, //Drive Motor
            config.backRightModuleSteerMotor_ID, //Steer Motor
            config.backRightModuleSteerEncoder_ID, //Steer Encoder
            config.drive.backRightModuleSteerOffset //Steer Offset
    );

    // We will also create a list of all the modules so we can easily access them later
    modules = new ArrayList<>(List.of(moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight));
  }

  public void orient() {
    // do something here.

  }

  public DrivetrainSubsystem(Config config) {
    super(config);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void simulationPeriodic() {
    //this.setStates(this.drivetrainModel.getSwerveModuleStates());
    //this.drivetrainModel.update(false, this.config.maxVoltage);
  }

  @Override
  public void periodic() {
      //drivetrainModel.setModuleStates(chassisSpeeds);
      this.drive(this.chassisSpeeds);

      this.dumpInfo();
  }

  public void setStates(SwerveModuleState[] states)
  {
    if (states != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity_metersPerSecond);

      for(int i = 0; i < 4; i++)
      {
        //System.out.println("Module " + i + ": " + states[i].angle.getDegrees());
        modules.get(i).set(states[i].speedMetersPerSecond / maxVelocity_metersPerSecond * this.config.maxVoltage, states[i].angle.getRadians());
      }

      pose = odometry.update(this.gyro.getRotation2d(), states[0], states[1], states[2], states[3]);
    }

    field.setRobotPose(pose);
  }

  //DOES NOT RESET GYRO
  public void setOdometry(Pose2d startingPosition) {
    odometry = new SwerveDriveOdometry(kinematics, this.gyro.getRotation2d(), startingPosition);

    odometryLoggable.log(LogLevel.DEBUG, "Reset Odometry to Starting Position: " + startingPosition);
    //this.dumpInfo();
  }

  public void zeroGyro()
  {
    gyro.reset();
  }

  public void resetGyroWithOffset(Rotation2d r)
  {
    this.zeroGyro();
    this.gyro.setAngleAdjustment(r.getDegrees());
  }

  public Rotation2d getGyroAngle() {
    return this.gyro.getRotation2d();
  }

  private void dumpInfo()
  {
    StringJoiner s = new StringJoiner("\n")
            .add("-----------------")
            .add("Robot Position: " + this.pose)
            .add("Odometry Position: " + this.odometry.getPoseMeters())
            .add("Drivetrain Gyro Heading: " + this.gyro.getRotation2d())
            .add("-----------------");

    odometryLoggable.log(LogLevel.DEBUG, s.toString());
  }

  public void zeroStates(Pose2d start)
  {
    SwerveModuleState zeroState = new SwerveModuleState(0, start.getRotation());
    SwerveModuleState[] states = {zeroState, zeroState, zeroState, zeroState};
    this.setStates(states);

    this.odometry.resetPosition(start, start.getRotation());
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
  
  @Override
  public void disable() {
    stop();
  }
  
}
