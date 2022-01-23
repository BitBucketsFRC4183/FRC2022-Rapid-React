package frc.robot.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class Config {

  // List of subsystem names:
  // Autonomous
  // Climber
  // Drive
  // Intake

  //////////////////////////////////////////////////////////////////////////////
  // Subsystem Enablers
  public boolean enableAutonomousSubsystem = true;
  public boolean enableClimberSubsystem = true;
  public boolean enableDriveSubsystem = true;
  public boolean enableIntakeSubsystem = true;

  //////////////////////////////////////////////////////////////////////////////
  // General Stuff
  public int maxVoltage = 12;

  // Motor IDs

  // Autonomous Subsystem

  // Climber Subsystem

  // Drive Subsystem
  public int frontLeftModuleDriveMotor = 5;
  public int frontLeftModuleSteerMotor = 6;
  public int frontLeftModuleSteerEncoder = 11;

  public int frontRightModuleDriveMotor = 7;
  public int frontRightModuleSteerMotor = 8;
  public int frontRightModuleSteerEncoder = 12;

  public int backLeftModuleDriveMotor = 3;
  public int backLeftModuleSteerMotor = 4;
  public int backLeftModuleSteerEncoder = 10;

  public int backRightModuleDriveMotor = 1;
  public int backRightModuleSteerMotor = 2;
  public int backRightModuleSteerEncoder = 9;

  // Intake Subsystem

  //////////////////////////////////////////////////////////////////////////////
  // Subsystem Configs
  public AutonomousConfig auto = new AutonomousConfig();
  public DriveConfig drive = new DriveConfig();

  // Autonomous Config
  public class AutonomousConfig {

    // Starting positions
    // Farleft X:6.3, Y:5, R:135
    public Pose2d farLeftStart = new Pose2d(new Translation2d(6.3, 5), Rotation2d.fromDegrees(135));
    // NearLeft
    // NearRight X:6.8, Y:2.8, R:210
    public Pose2d nearRightStart = new Pose2d(new Translation2d(6.8, 2.8), Rotation2d.fromDegrees(210));
    // FarRight

    // Trajectories
    public Trajectory farLeftStartTrajectory = TrajectoryGenerator.generateTrajectory(
      farLeftStart,
      List.of(new Translation2d(4.9, 6.4), new Translation2d(4.3, 5.7)),
      new Pose2d(7.25, 4.5, Rotation2d.fromDegrees(-25)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
    );
    public Trajectory nearRightStartTrajectory = TrajectoryGenerator.generateTrajectory(
      nearRightStart,
      List.of(new Translation2d(4.9, 1.9), new Translation2d(7, 1.7)),
      new Pose2d(7.8, 3.2, Rotation2d.fromDegrees(65)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
    );

    public AutonomousConfig() {}
  }

  // Climber Config
  public class ClimberConfig {

    public ClimberConfig() {}
  }

  // Drive Config
  public class DriveConfig {

    public double drivetrainTrackWidth_meters = 0.3937; // set trackwidth

    public double drivetrainWheelBase_meters = 0.3937; // set wheelbase

    public double frontLeftModuleSteerOffset = -Math.toRadians(345); // set front left steer offset

    public double frontRightModuleSteerOffset = -Math.toRadians(149); // set front right steer offset

    public double backLeftModuleSteerOffset = -Math.toRadians(321); // set back left steer offset

    public double backRightModuleSteerOffset = -Math.toRadians(60.9); // set back right steer offset

    public DriveConfig() {
    }
  }

  // Intake Config
  public class IntakeConfig {

    public IntakeConfig() {}
  }

  public Config() {}
}
