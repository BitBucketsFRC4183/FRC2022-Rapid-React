package frc.robot.config;

import frc.robot.config.MotorConfig.EncoderType;

public class Config {

  // List of subsystem names:
  // Autonomous
  // Climber
  // Drive
  // Intake
  // Shooter
  // Vision

  //////////////////////////////////////////////////////////////////////////////
  // Enablers
  public boolean enableAutonomousSubsystem = true;
  public boolean enableClimberSubsystem = true;
  public boolean enableDriveSubsystem = true;
  public boolean enableIntakeSubsystem = true;
  public boolean enableRGBSubsystem = true;
  public boolean enableShooterSubsystem = true;
  public boolean enableVisionSubsystem = false;

  public boolean enablePneumatics = true;

  //////////////////////////////////////////////////////////////////////////////
  // General Stuff
  public int maxVoltage = 12;

  // Motor & Pneumatic IDs

  // Autonomous Subsystem

  // Drive Subsystem
  public int frontLeftModuleDriveMotor_ID = 1;
  public int frontLeftModuleSteerMotor_ID = 2;
  public int frontLeftModuleSteerEncoder_ID = 9;

  public int frontRightModuleDriveMotor_ID = 7;
  public int frontRightModuleSteerMotor_ID = 8;
  public int frontRightModuleSteerEncoder_ID = 12;

  public int backLeftModuleDriveMotor_ID = 5;
  public int backLeftModuleSteerMotor_ID = 6;
  public int backLeftModuleSteerEncoder_ID = 11;

  public int backRightModuleDriveMotor_ID = 3;
  public int backRightModuleSteerMotor_ID = 4;
  public int backRightModuleSteerEncoder_ID = 10;

  // Intake Subsystem
  public static int ballManagementMotor_ID = 13;
  public static int intakeMotor_ID = 18;

  // Shooter
  public int shooterTop_ID = 15;
  public int shooterBottom_ID = 14;

  public int shooterFeeder_ID = 19;

  public int intakeSolenoid_ID1 = 0;
  public int intakeSolenoid_ID2 = 1;

  // Shooter

  //Climber Subsystem
  public int climberMotor_IDLeft = 16;
  public int climberMotor_IDRight = 17;

  public int elevatorSolenoid_ID1 = 2;
  public int elevatorSolenoid_ID2 = 3;

  //RGB
  public static int RGB_ID = 9;

  //////////////////////////////////////////////////////////////////////////////
  // Subsystem Configs
  public AutonomousConfig auto = new AutonomousConfig();
  public DriveConfig drive = new DriveConfig();
  public IntakeConfig intake = new IntakeConfig();
  public RGBConfig rgbConfig = new RGBConfig();
  public ShooterConfig shooter = new ShooterConfig();
  public VisionConfig vision = new VisionConfig();
  public ClimberConfig climber = new ClimberConfig();

  // Autonomous Config
  public class AutonomousConfig {

    public String nothingPath = "Nothing";

    public double maxPathFollowVelocity = 1.5;
    public double maxPathFollowAcceleration = 2.5;

    //public PID pathXYPID = new PID(2.2956, 0, 0);
    public PID pathXYPID = new PID(3.2416, 0, 0);
    public PID pathThetaPID = new PID(3, 0, 0.02);

    public AutonomousConfig() {}
  }

  // Climber Config
  public class ClimberConfig {

    public MotorConfig climberLeft = new MotorConfig();
    public MotorConfig climberRight = new MotorConfig();

    public ClimberConfig() {}
  }

  // rgb Config
  public class RGBConfig {

    public RGBConfig() {}
  }

  // Drive Config
  public class DriveConfig {

    public double robotWeight_pounds = 70.0;

    public double drivetrainTrackWidth_meters = 0.6096; // set trackwidth

    public double drivetrainWheelBase_meters = 0.7112; // set wheelbase

    public double frontLeftModuleSteerOffset = -Math.toRadians(232.55); // set front left steer offset

    public double frontRightModuleSteerOffset = -Math.toRadians(331.96 - 180); // set front right steer offset

    public double backLeftModuleSteerOffset = -Math.toRadians(255.49); // set back left steer offset

    public double backRightModuleSteerOffset = -Math.toRadians(70.66 + 180); // set back right steer offset

    public DriveConfig() {}
  }

  // Intake Config
  public class IntakeConfig {

    public MotorConfig intakeMotor = new MotorConfig();
    public MotorConfig ballManagementMotor = new MotorConfig();

    public boolean defaultIntakeAutoExtend = false;

    public IntakeConfig() {}
  }

  // Shooter Config
  public class ShooterConfig {

    public MotorConfig shooterTop = new MotorConfig();
    public MotorConfig shooterBottom = new MotorConfig();
    public MotorConfig feeder = new MotorConfig();

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

    // Intake
    intake.intakeMotor.id = intakeMotor_ID;
    intake.intakeMotor.inverted = true;

    intake.ballManagementMotor.id = ballManagementMotor_ID;
    intake.ballManagementMotor.inverted = true;

    // Shooter
    // For FeedFoward: 7 V (out of 12) gave us 56/54 rotations / second during characterization
    // we found it was a bit more accurate if we went with 60 rot/s, hence the 60*60 below
    // For kI, the rev docs (https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control)
    // say that kI is multiplied by the error and added to output for each pid loop. I think these pid loops run at 1khz,
    // so divide the kI by 1000 otherwise it's too large.
    shooter.shooterTop.id = shooterTop_ID;
    shooter.shooterTop.velocityPIDF = new PIDF(/*P*/0.0001 / 4, /*I*/0.001 / 1000, /*D*/0, /*F*/((7.0 / 12.0) / (60 * 60)) * (12.0/11.0), /*izone*/300);
    shooter.shooterTop.inverted = true;

    shooter.shooterBottom.id = shooterBottom_ID;
    shooter.shooterBottom.velocityPIDF = new PIDF(/*P*/0.0001 / 4, /*I*/0.001 / 1000, /*D*/0, /*F*/((7.0 / 12.0) / (60 * 60)) * (12.0/11.0), /*izone*/300);
    shooter.shooterBottom.inverted = false;

    shooter.feeder.id = shooterFeeder_ID;
    shooter.feeder.inverted = true;
    
    ///////////////////
    // climber motors
    climber.climberLeft.id = climberMotor_IDLeft;
    climber.climberLeft.encoderType = EncoderType.Quadrature;
    // TODO: actually tune these
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#motion-magic-position-velocity-current-closed-loop-closed-loop
    climber.climberLeft.motionMagicCruiseVelocity = 19000;
    climber.climberLeft.motionMagicAcceleration = 10000;

    // TODO:
    // What's the difference between the two sorts of peak outputs? Add a comment. Also, it might be nice to write down the encoder step to inch (and time unit to second) conversion in a comment.
    // Oh, I think it's the primary and aux PID output limits, right? Maybe you should call it that, not distance/ turning; this isn't a drivetrain that turns
    climber.climberLeft.positionPIDF = new PIDF(/*P*/0.1, /*I*/0, /*D*/0, /*F*/0.00018);
    climber.climberLeft.inverted = false; // whether it should go forward or backward given some voltage
    climber.climberLeft.sensorPhase = true; // whether going forward counts as positive or negative ticks to the encoder
    climber.climberLeft.distancePeakOutput = 0.5;
    climber.climberLeft.turningPeakOutput = 1;

    climber.climberRight.id = climberMotor_IDRight;
    climber.climberRight.encoderType = EncoderType.Quadrature;
    // TODO: actually tune these
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#motion-magic-position-velocity-current-closed-loop-closed-loop
    climber.climberRight.motionMagicCruiseVelocity = 19000;
    climber.climberRight.motionMagicAcceleration = 10000;
    climber.climberRight.positionPIDF = new PIDF(/*P*/0.1, /*I*/0, /*D*/0, /*F*/0.00018);
    climber.climberRight.inverted = false;
    climber.climberRight.sensorPhase = false;
    climber.climberRight.distancePeakOutput = 0.5;
    climber.climberRight.turningPeakOutput = 1;
  }
}
