package frc.robot.config;

public class Config {

  // List of subsystem names:
  // Autonomous
  // Climber
  // Drive
  // Intake
  // Shooter
  // Vision

  //////////////////////////////////////////////////////////////////////////////
  // Subsystem Enablers
  public boolean enableAutonomousSubsystem = true;
  public boolean enableClimberSubsystem = true;
  public boolean enableDriveSubsystem = true;
  public boolean enableIntakeSubsystem = true;
  public boolean enableShooterSubsystem = true;
  public boolean enableVisionSubsystem = true;

  //////////////////////////////////////////////////////////////////////////////
  // General Stuff
  public int maxVoltage = 12;

  // Motor IDs

  // Autonomous Subsystem

  // Climber Subsystem

  // Drive Subsystem
  public int frontLeftModuleDriveMotor_ID = 5;
  public int frontLeftModuleSteerMotor_ID = 6;
  public int frontLeftModuleSteerEncoder_ID = 11;

  public int frontRightModuleDriveMotor_ID = 7;
  public int frontRightModuleSteerMotor_ID = 8;
  public int frontRightModuleSteerEncoder_ID = 12;

  public int backLeftModuleDriveMotor_ID = 3;
  public int backLeftModuleSteerMotor_ID = 4;
  public int backLeftModuleSteerEncoder_ID = 10;

  public int backRightModuleDriveMotor_ID = 1;
  public int backRightModuleSteerMotor_ID = 2;
  public int backRightModuleSteerEncoder_ID = 9;

  // Intake Subsystem
  public static int intakeMotor_ID = 13;

  public int shooterRoller1_ID = 14;
  public int shooterRoller2_ID = 15;

  // Shooter

  //Climber Subsystem
  public static int climberMotor_ID = 16;

  //////////////////////////////////////////////////////////////////////////////
  // Subsystem Configs
  public AutonomousConfig auto = new AutonomousConfig();
  public DriveConfig drive = new DriveConfig();
  public ShooterConfig shooter = new ShooterConfig();
  public VisionConfig vision = new VisionConfig();

  // Autonomous Config
  public class AutonomousConfig {

    public String nothingPath = "Nothing";
    public String genericPath = "Path";
    public String driveBackwardsPath = "Drive Backwards";

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

    public DriveConfig() {}
  }

  // Intake Config
  public class IntakeConfig {

    public IntakeConfig() {}
  }

  // Shooter Config
  public class ShooterConfig {

    public MotorConfig roller1 = new MotorConfig();
    public MotorConfig roller2 = new MotorConfig();

    public ShooterConfig() {}
  }

  public class VisionConfig {

    public double targetHeight = 0;
    public double cameraHeight = 0;
    public double verticalCameraAngle = 0;

    public VisionConfig() {}
  }

  public Config() {
    //////////////////////////////////////////////////////////////////////////////
    // Motor Configuration

    shooter.roller1.id = shooterRoller1_ID;
    shooter.roller1.velocityPIDF = new PIDF(/*P*/0.00001, /*I*/0, /*D*/0, /*F*/0.00018);

    shooter.roller2.id = shooterRoller2_ID;
    shooter.roller2.velocityPIDF = new PIDF(/*P*/0.00001, /*I*/0, /*D*/0, /*F*/0.00018);
  }
}
