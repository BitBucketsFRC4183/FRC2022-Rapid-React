package frc.robot.config;

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
    // Motor IDs
 
    // Autonomous Subsystem

    // Climber Subsystem

    // Drive Subsystem
    public int frontLeftModuleDriveMotor = 5; 
    public int frontLeftModuleSteerMotor = 6; 
    public int frontLeftModuleSteerEncoder = 11;

    public int frontRightModuleDriveMotor = 7;
    public int frontRightModuleSteerMotor = 8; 
    public int frontRightModuleSteerEncoder = 12 ; 

    public int backLeftModuleDriveMotor = 3; 
    public int backLeftModuleSteerMotor = 4;
    public int backLeftModuleSteerEncoder = 10; 

    public int backRightModuleDriveMotor = 1; 
    public int backRightModuleSteerMotor = 2; 
    public int backRightModuleSteerEncoder = 9; 
    // Intake Subsystem

    //////////////////////////////////////////////////////////////////////////////
    // Subsystem Configs
    public DriveConfig drive = new DriveConfig();
    // Autonomous Config
    public class AutonomousConfig {
        public AutonomousConfig() {

        }
    }

    // Climber Config
    public class ClimberConfig {
        public ClimberConfig() {

        }
    }

    // Drive Config
    public class DriveConfig {
        public double DRIVETRAIN_TRACKWIDTH_METERS = 0.3937; //set trackwidth

        public  double DRIVETRAIN_WHEELBASE_METERS = 0.3937; //set wheelbase

        public  double frontLeftModuleSteerOffset = -Math.toRadians(345); //set front left steer offset

        public  double frontRightModuleSteerOffset = -Math.toRadians(149); //set front right steer offset


        public  double backLeftModuleSteerOffset = -Math.toRadians(321); //set back left steer offset

        public  double backRightModuleSteerOffset = -Math.toRadians(60.9); //set back right steer offset
    


        public DriveConfig() 
        {

           

        }
    }

    // Intake Config
    public class IntakeConfig {
        public IntakeConfig() {

        }
    }

    public Config() {
    }
}
