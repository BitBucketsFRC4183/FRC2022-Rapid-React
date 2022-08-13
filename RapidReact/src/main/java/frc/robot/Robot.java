// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.config.Config;
import frc.robot.simulator.CTREPhysicsSim;
import frc.robot.subsystem.other.*;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.AutonomousPath;
import frc.robot.utils.MathUtils;

import java.util.*;
import java.util.function.Supplier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  private final Loggable<String> info = BucketLog.loggable(Put.STRING, "general/info");

  private Buttons buttons;
  private Config config;

  private final List<BitBucketsSubsystem> robotSubsystems = new ArrayList<>();

  private AutonomousSubsystem autonomousSubsystem;
  private DrivetrainSubsystem drivetrainSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private RGBSubsystem rgbSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private VisionSubsystem visionSubsystem;

  private HoodSubsystem hoodSubsystem;
  private Field2d field;
  private ClimberSubsystem climberSubsystem;

  private SendableChooser<AutonomousPath> autonomousChooser = new SendableChooser<>();
  private Map<AutonomousPath, AutonomousCommand> autonomousCommands = new HashMap<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    this.config = new Config();
    this.buttons = new Buttons();
    this.field = new Field2d();

    LiveWindow.disableAllTelemetry();

    //Autonomous Selection
    Arrays.stream(AutonomousPath.values()).forEach(path -> this.autonomousChooser.addOption(path.dashboardName, path));
    this.autonomousChooser.setDefaultOption(AutonomousPath.NOTHING.dashboardName, AutonomousPath.NOTHING);
    SmartDashboard.putData("Autonomous Path Chooser", this.autonomousChooser);

    // Add Subsystems Here
    if (config.enableAutonomousSubsystem) {
      this.robotSubsystems.add(autonomousSubsystem = new AutonomousSubsystem(this.config));
    }
    if (config.enableRGBSubsystem) {
      this.robotSubsystems.add(rgbSubsystem = new RGBSubsystem());
    }
    if (config.enableDriveSubsystem) {
      this.robotSubsystems.add(drivetrainSubsystem = new DrivetrainSubsystem(this.config));
    }
    if (config.enableIntakeSubsystem) {
      this.robotSubsystems.add(intakeSubsystem = new IntakeSubsystem(this.config));
    }
    if (config.enableShooterSubsystem) {
      this.robotSubsystems.add(shooterSubsystem = new ShooterSubsystem(this.config));
    }
    if (config.enableClimberSubsystem) {
      this.robotSubsystems.add(climberSubsystem = new ClimberSubsystem(this.config));
    }

    this.robotSubsystems.add(visionSubsystem = new VisionSubsystem(this.config, angleDegrees, heightInches));
    this.robotSubsystems.add(hoodSubsystem = new HoodSubsystem(this.config));

    // create a new field to update
    SmartDashboard.putData("Field", field);

    // Configure the button bindings
    this.configureButtonBindings();

    // Subsystem Initialize Loop
    if (System.getenv().containsKey("CI")) {
      this.robotSubsystems.add(new LogTestSubsystem(this.config));
      this.robotSubsystems.add(new SimulatorTestSubsystem(this.config));
    }

    this.robotSubsystems.add(new SetModeTestSubsystem(this.config));

    // Subsystem Initialize Loop

    this.robotSubsystems.forEach(BitBucketsSubsystem::init);

    //Create the Autonomous Commands now so we don't do this every time autonomousInit() gets called
    this.cacheAutonomousCommands();
  }

  private void cacheAutonomousCommands()
  {
    Supplier<AutonomousCommand> base = () -> new AutonomousCommand(
          this.autonomousSubsystem,
          this.drivetrainSubsystem,
          this.intakeSubsystem,
          this.shooterSubsystem,
          this.rgbSubsystem
    );

    //Just to grab the getCommand methods
    final AutonomousCommand instance = base.get();

    this.autonomousCommands.put(AutonomousPath.NOTHING, base.get()
            .executeDrivePath(AutonomousPath.NOTHING, 0)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.HARDCODED, base.get()
            .shootOne(true)
            .executeAction((d, i, s) -> {
              i.forceIntaking();
              i.spinForward();
              s.antiFeed();
            }, 0.2)
            .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(1.5, 0.0, 0)), 1) //Drive out of the tarmac
            .executeAction((d, i, s) -> d.stop(), 2.0) //Drive out of the tarmac pt 2
            .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(-1.5, 0.0, 0)), 2) //Drive back to the hub
            .executeAction((d, i, s) -> d.stop(), 2.5) //Drive back to the hub pt 2
            .executeAction((d, i, s) -> d.stop(), .5) //Drive back to the hub pt 2
            .shootOne(false)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.ONE_BALL_PATHPLANNER, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.ONE_BALL_PATHPLANNER, 0.5)
            .stop()
    );

    //One Ball Hardcoded
//    this.autonomousCommands.put(AutonomousPath.ONE_BALL_HC, base.get()
//            .shootOne(true)
//            .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(1.5, 0.0, 0)), 0)
//            .executeAction((d, i, s) -> d.stop(), 2.0)
//            .stop()
//    );
    this.autonomousCommands.put(AutonomousPath.ONE_BALL_HC, base.get()
            .executeAction((d, i, s) -> {
              s.autoTopSpeedHighOffset = 800;
              s.autoBottomSpeedHighOffset = 0; //100
            }, 0.02)
            .shootOne(true)
            .executeDrivePath("1 Ball Taxi", 0.01)
            .stop()
     );

    this.autonomousCommands.put(AutonomousPath.ONE_BALL_INTAKE, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.ONE_BALL_INTAKE, 1, instance.getDropIntakeCommand())//, command.getSpinShooterCommand(true))
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.TWO_BALL_HANGAR, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.TWO_BALL_HANGAR, 1, instance.getDropIntakeCommand())//, command.getSpinShooterCommandWithDelay(true, 0.5))
            .shootOne(true)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.TWO_BALL_WALL, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.TWO_BALL_WALL, 1, instance.getDropIntakeCommand())//, command.getSpinShooterCommandWithDelay(true, 0.5))
            .shootOne(true)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.THREE_BALL, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.THREE_BALL, 0.02, instance.getDropIntakeCommand())//, command.getSpinShooterCommandWithDelay(true, 2.0))
            .shootTwo(true)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.FOUR_BALL, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.THREE_BALL, 0.05, instance.getDropIntakeCommand())
            .shootTwo(true)
            .executeDrivePath("4 Ball Auto P2", 0.0)
            .executeDrivePath("4 Ball Auto P3", 0.2)//, command.getSpinShooterCommand(true))
            .shootOne(true)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.FOUR_BALL_ALT, base.get()
            .executeDrivePath(AutonomousPath.TWO_BALL_WALL, 0.05, instance.getDropIntakeCommand())
            .shootTwo(true)
            .executeDrivePath("4 Ball Auto Alt P2", 0.04)
            .shootTwo(true)
            .stop()
    );

    this.autonomousCommands.put(AutonomousPath.FIVE_BALL, base.get()
            .shootOne(true)
            .executeDrivePath(AutonomousPath.THREE_BALL, 1, instance.getDropIntakeCommand())
            .shootTwo(true)
            .executeDrivePath("4 Ball Auto P2", 1)
            .executeDrivePath("4 Ball Auto P3", 3)//, command.getSpinShooterCommand(true))
            .shootOne(true)
            .stop()
    );
  }

  boolean wasShooting;

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //this.robotSubsystems.forEach(BitBucketsSubsystem::periodic);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    if (config.enableDriveSubsystem && config.enableAutonomousSubsystem) {
      this.info.log(LogLevel.GENERAL, "auton started");

      long timeI = System.currentTimeMillis();

      AutonomousCommand command = this.autonomousCommands.getOrDefault(this.autonomousChooser.getSelected(), this.autonomousCommands.get(AutonomousPath.NOTHING));

      SmartDashboard.putNumber("autonomousInit_time", System.currentTimeMillis() - timeI);

      command.schedule();

      //CommandScheduler.getInstance().schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    info.log(LogLevel.GENERAL, "Still in autonomous");

    //System.out.println("Odometry Position: " + this.drivetrainSubsystem.odometry.getPoseMeters());
    //System.out.println("Gyro Heading: " + this.drivetrainSubsystem.gyro.getRotation2d());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    shooterSubsystem.disable();
    intakeSubsystem.disable();

    this.shooterSubsystem.autoBottomSpeedHighOffset = 0;
    this.shooterSubsystem.autoTopSpeedHighOffset = 0;

    if (config.enableDriveSubsystem) {
      drivetrainSubsystem.setDefaultCommand(
          new DefaultDriveCommand(
          drivetrainSubsystem,
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveForward)),
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveStrafe)),
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveRotation))
        )
      );
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if((int)Timer.getMatchTime() == 30) this.rgbSubsystem.alertMatchTimeLeft();

    if(!this.shooterSubsystem.isAutoShooting)
    {
      if (shooterSubsystem.isShooting()) {
        wasShooting = true;
        if (shooterSubsystem.isUpToHighSpeed()) {
          rgbSubsystem.upToSpeed();
        } else {
          rgbSubsystem.notUpToSpeed();
        }
      } else {
        if (wasShooting) {
          rgbSubsystem.normalize();
          wasShooting = false;
        }
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    this.robotSubsystems.forEach(BitBucketsSubsystem::disable);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    this.drivetrainSubsystem.stop();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    drivetrainSubsystem.stop();
  }

  @Override
  public void simulationPeriodic() {
    CTREPhysicsSim.getInstance().run();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {XboxController}), and then passing it to
   * a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    if (config.enableDriveSubsystem) {
      buttons.resetOdometry.whenPressed(
        () -> {
          this.drivetrainSubsystem.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
          this.drivetrainSubsystem.zeroGyro();
        }
      );

      buttons.slowDrive
        .whenPressed(() -> {
          if(this.drivetrainSubsystem.speedModifier > 0.25) this.drivetrainSubsystem.speedModifier = 0.25;
          else this.drivetrainSubsystem.speedModifier = 0.75;
        });
        //.whenReleased(() -> this.drivetrainSubsystem.speedModifier = .75);
    }

    //Intake buttons
    if (config.enableIntakeSubsystem) {
      buttons.intake.whenPressed(intakeSubsystem::spinForward);
      buttons.outtake.whenPressed(intakeSubsystem::spinBackward);
      buttons.intake.whenReleased(intakeSubsystem::stopSpin);
      buttons.outtake.whenReleased(intakeSubsystem::stopSpin);
      buttons.toggleIntake.whenPressed(intakeSubsystem::toggle);
    }

    if (config.enableIntakeSubsystem && config.enableShooterSubsystem) {
      buttons.intake.whenPressed(
        () -> {
          shooterSubsystem.antiFeed();
          intakeSubsystem.spinForward();
        }
      );
      buttons.intake.whenReleased(
        () -> {
          shooterSubsystem.turnOffFeeders();
        }
      );
    }

    //Shooter Buttons
    if (config.enableShooterSubsystem) {
      buttons.lowShoot.whenPressed(shooterSubsystem::shootLow);
      buttons.lowShoot.whenReleased(shooterSubsystem::stopShoot);

      buttons.hubSpinUp.whenPressed(
        () -> {
          shooterSubsystem.spinUpTop();
        }
      );
      buttons.hubSpinUp.whenReleased(
        () -> {
          shooterSubsystem.stopShoot();
          if (config.enableIntakeSubsystem) {
            intakeSubsystem.stopBallManagement();
          }
        }
      );

      buttons.autoShoot.whenHeld(new AutoShootCommand(this.shooterSubsystem, this.intakeSubsystem, this.rgbSubsystem).withParameters(2, true));

      buttons.autoShootOne.whenHeld(new AutoShootCommand(this.shooterSubsystem, this.intakeSubsystem, this.rgbSubsystem).withParameters(1, true));

      buttons.feedInFire.whenPressed(
        () -> {
          shooterSubsystem.turnOnFeeders();
          intakeSubsystem.ballManagementForward();
        }
      );
      buttons.feedInFire.whenReleased(
        () -> {
          shooterSubsystem.turnOffFeeders();
          intakeSubsystem.stopBallManagement();
        }
      );
    }

    //Climber buttons
    if (config.enableClimberSubsystem) {
      buttons.toggleElevator.whenPressed(() -> climberSubsystem.elevatorToggle());

      buttons.elevatorExtend.whenPressed(
        () -> {
          rgbSubsystem.climberEnabled();
          climberSubsystem.manualElevatorExtend();
        }
      );
      buttons.elevatorExtend.whenReleased(
        () -> {
          rgbSubsystem.normalize();
          climberSubsystem.elevatorStop();
        }
      );

      buttons.elevatorRetract.whenPressed(
        () -> {
          rgbSubsystem.climberEnabled();
          climberSubsystem.manualElevatorRetract();
        }
      );
      buttons.elevatorRetract.whenReleased(
        () -> {
          rgbSubsystem.normalize();
          climberSubsystem.elevatorStop();
        }
      );
    }

    buttons.rgb.whenPressed(
      () -> {
        rgbSubsystem.funnyButton();
      }
    );
  }
}
