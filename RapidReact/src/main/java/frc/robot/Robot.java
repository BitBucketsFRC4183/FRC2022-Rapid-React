// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.AutonomousFollowPathCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.simulator.CTREPhysicsSim;
import frc.robot.simulator.SetModeTestSubsystem;
import frc.robot.simulator.SimulatorTestSubsystem;
import frc.robot.subsystem.*;
import frc.robot.utils.AutonomousPath;
import frc.robot.utils.MathUtils;

import java.util.ArrayList;
import java.util.List;

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
  private Field2d field;
  private ClimberSubsystem climberSubsystem;
  private boolean driverClimbEnabledPressed;
  private boolean operatorClimbEnabledPressed;

  private boolean autoClimbStopLeftPressed;
  private boolean autoClimbStopRightPressed;

  private SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

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

    this.autonomousPathChooser.addOption("Nothing", AutonomousPath.NOTHING);
    this.autonomousPathChooser.addOption("Hardcoded: Shoot Preload, Drive Back Left", AutonomousPath.HARDCODED_SHOOT_AND_DRIVE_BACK_LEFT);
    this.autonomousPathChooser.addOption("Hardcoded: Shoot Preload, Drive Back Right", AutonomousPath.HARDCODED_SHOOT_AND_DRIVE_BACK_RIGHT);
    this.autonomousPathChooser.addOption("Hardcoded: Shoot Preload, Drive Back and Shoot Loaded Left", AutonomousPath.HARDCODED_SHOOT_DRIVE_BACK_AND_SHOOT);
    this.autonomousPathChooser.addOption("PathPlanner: Drive Backwards", AutonomousPath.PATH_PLANNER_DRIVE_BACKWARDS);
    this.autonomousPathChooser.addOption("PathPlanner: Shoot Preload and Drive Backwards", AutonomousPath.PATH_PLANNER_SHOOT_AND_DRIVE_BACKWARDS);
    this.autonomousPathChooser.addOption("PathPlanner: Shoot Preload, Intake Two Balls", AutonomousPath.PATH_PLANNER_SHOOT_INTAKE_TWO_BALLS);
    this.autonomousPathChooser.addOption("PathPlanner: Main - No Terminal", AutonomousPath.MAIN_NO_TERMINAL);
    this.autonomousPathChooser.addOption("PathPlanner: Main - With Terminal", AutonomousPath.MAIN_WITH_TERMINAL);

    this.autonomousPathChooser.setDefaultOption("Default (Nothing)", AutonomousPath.NOTHING);

    SmartDashboard.putData("Autonomous Path Chooser", this.autonomousPathChooser);

    // Add Subsystems Here
    if (config.enableAutonomousSubsystem) {
      this.robotSubsystems.add(autonomousSubsystem = new AutonomousSubsystem(this.config));
    }
    if (config.enableRGBSubsystem) {
      this.robotSubsystems.add(rgbSubsystem = new RGBSubsystem(this.config));
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
  }

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

      Command command;
      switch (this.autonomousPathChooser.getSelected()) {
        case NOTHING:
          command =
            new AutonomousFollowPathCommand(
              config.auto.nothingPath,
              this.autonomousSubsystem,
              this.drivetrainSubsystem
            );
          break;
        case PATH_PLANNER_DRIVE_BACKWARDS:
          command =
            new AutonomousFollowPathCommand(
              config.auto.driveBackwardsPath,
              this.autonomousSubsystem,
              this.drivetrainSubsystem
            );
          break;
        case HARDCODED_SHOOT_AND_DRIVE_BACK_LEFT:
          drivetrainSubsystem.resetGyroWithOffset(Rotation2d.fromDegrees(150));
          command =
            new AutonomousCommand(
              this.autonomousSubsystem,
              this.drivetrainSubsystem,
              this.intakeSubsystem,
              this.shooterSubsystem
            )
              .executeShootPreload() //Shoot Preload
              .executeAction((d, i, s) -> i.spinForward()) //Activate Intake
              .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(3.0, 0.0, 0.0)), 1) //Drive out of the tarmac
              .executeAction((d, i, s) -> d.stop(), 1.0) //Drive out of the tarmac pt 2
              .complete();
          break;
        case HARDCODED_SHOOT_AND_DRIVE_BACK_RIGHT:
          drivetrainSubsystem.resetGyroWithOffset(Rotation2d.fromDegrees(-150));
          command =
            new AutonomousCommand(
              this.autonomousSubsystem,
              this.drivetrainSubsystem,
              this.intakeSubsystem,
              this.shooterSubsystem
            )
              .executeShootPreload() //Shoot Preload
              .executeAction((d, i, s) -> i.spinForward()) //Activate Intake
              .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(3.0, 0.0, 0.0)), 1) //Drive out of the tarmac
              .executeAction((d, i, s) -> d.stop(), 1.0) //Drive out of the tarmac pt 2
              .complete();
          break;
        case HARDCODED_SHOOT_DRIVE_BACK_AND_SHOOT:
          drivetrainSubsystem.resetGyroWithOffset(Rotation2d.fromDegrees(150));
          command =
            new AutonomousCommand(
              this.autonomousSubsystem,
              this.drivetrainSubsystem,
              this.intakeSubsystem,
              this.shooterSubsystem
            )
              .executeShootPreload() //Shoot Preload
              .executeAction((d, i, s) -> i.spinForward()) //Activate Intake
              .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(3.0, 0.0, 0.0)), 1) //Drive out of the tarmac
              .executeAction((d, i, s) -> d.stop(), 1.0) //Drive out of the tarmac pt 2
              .executeAction((d, i, s) -> d.drive(new ChassisSpeeds(-3.0, 0.0, 0.0)), 2) //Drive back to the hub
              .executeAction((d, i, s) -> d.stop(), 1.0) //Drive back to the hub pt 2
              .executeShootPreload()
              .complete();
          break;
        case PATH_PLANNER_SHOOT_AND_DRIVE_BACKWARDS:
          command =
           new AutonomousCommand(
             this.autonomousSubsystem,
             this.drivetrainSubsystem,
             this.intakeSubsystem,
             this.shooterSubsystem
           )
             .executeShootPreload()
             .executeAction((d, i, s) -> i.spinForward())
             .executeDrivePath("Drive Backwards Single Ball", 1)
             .executeAction((d, i, s) -> i.stopSpin(), 2)
             .complete();
          break;
        case PATH_PLANNER_SHOOT_INTAKE_TWO_BALLS:
          command =
           new AutonomousCommand(
             this.autonomousSubsystem,
             this.drivetrainSubsystem,
             this.intakeSubsystem,
             this.shooterSubsystem
           )
             .executeShootPreload()
             .executeDrivePath("Drive Backwards Double Ball P1")
             .executeAction((d, i, s) -> i.spinForward())
             .executeDrivePath("Drive Backwards Double Ball P2", 2)
             .executeAction((d, i, s) -> i.stopSpin(), 2)
             .complete();
          break;
        case MAIN_NO_TERMINAL:
          command =
            new AutonomousCommand(
              this.autonomousSubsystem,
              this.drivetrainSubsystem,
              this.intakeSubsystem,
              this.shooterSubsystem
            )
              .executeShootPreload() //Shoot Preload
              .executeDrivePath("Main P1") //Drive to the first ball
              .executeAction((d, i, s) -> i.spinForward()) //Activate intake
              .executeDrivePath("Main P2 Ball", 2.0) //Skip terminal, go straight to the second ball
              .executeAction((d, i, s) -> i.spinBackward(), 2.0) //Turn off the intake after getting the ball
              .executeDrivePath("Main P3") //Drive to the base of the hub
              .executeAction((d, i, s) -> s.spinUpTop()) //Shoot - Spin up Top
              .executeAction((d, i, s) -> {
                s.turnOnFeeders(); //Activate feeders
                i.ballManagementForward(); //Activate BMS in case a ball doesn't get pulled by the feeders
              }, 2) //Wait 2 seconds for the shooter to spin up
              .complete();
          break;
        case MAIN_WITH_TERMINAL:
          command =
            new AutonomousCommand(
              this.autonomousSubsystem,
              this.drivetrainSubsystem,
              this.intakeSubsystem,
              this.shooterSubsystem
            )
              .executeShootPreload() //Shoot Preload
              .executeDrivePath("Main P1") //Drive to the first ball
              .executeAction((d, i, s) -> i.spinForward()) //Activate intake
              .executeDrivePath("Main P2 Terminal", 2.0) //Head to the Terminal ball and push it in
              .executeDrivePath("Main P2.5 Terminal") //Head to the second ball
              .executeAction((d, i, s) -> i.spinBackward(), 2.0) //Turn off the intake after getting the ball
              .executeDrivePath("Main P3") //Drive to the base of the hub
              .executeAction((d, i, s) -> s.spinUpTop()) //Shoot - Spin up Top
              .executeAction((d, i, s) -> {
                s.turnOnFeeders(); //Activate feeders
                i.ballManagementForward(); //Activate BMS in case a ball doesn't get pulled by the feeders
              }, 2) //Wait 2 seconds for the shooter to spin up
              .complete();
          break;
        default:
          info.log(
            LogLevel.CRITICAL,
            "Invalid Autonomous Path! (SendableChooser Output: " + this.autonomousPathChooser.getSelected() + ")"
          );

          return;
      }

      command.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    info.log(LogLevel.GENERAL, "Still in autonomous");
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit()
  {
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
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    this.robotSubsystems.forEach(BitBucketsSubsystem::disable);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

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
      buttons.resetOdometry.whenPressed(() -> {
        this.drivetrainSubsystem.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        this.drivetrainSubsystem.zeroGyro();
      });

      buttons.slowDrive
              .whenPressed(() -> this.drivetrainSubsystem.speedModifier = 0.5)
              .whenReleased(() -> this.drivetrainSubsystem.speedModifier = 1.0);
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

      buttons.hubSpinUp.whenPressed(() -> {
        shooterSubsystem.spinUpTop();
      });
      buttons.hubSpinUp.whenReleased(() -> {
        shooterSubsystem.stopShoot();
        if (config.enableIntakeSubsystem) {
          intakeSubsystem.stopBallManagement();
        }
      });

      buttons.feedInFire.whenPressed(() -> {
        shooterSubsystem.turnOnFeeders();
        intakeSubsystem.ballManagementForward();
      });
      buttons.feedInFire.whenReleased(() -> {
        shooterSubsystem.turnOffFeeders();
        intakeSubsystem.stopBallManagement();
      });
    
      buttons.tarmacShootOrToggleElevator.whenPressed(
        () -> {
          if (config.enableClimberSubsystem && climberSubsystem.isClimberEnabled()) {
            climberSubsystem.elevatorToggle();
          } else {
            shooterSubsystem.shootTarmac();
            //lmao
            if (drivetrainSubsystem != null) {
              drivetrainSubsystem.orient();
            }
          }
        }
      );
      buttons.tarmacShootOrToggleElevator.whenReleased(
        () -> {
          if (config.enableShooterSubsystem) {
            shooterSubsystem.stopShoot();
          }
        }
      );
    }

    //Climber buttons
    if (config.enableClimberSubsystem) {
      buttons.operatorEnableClimber
        .whenPressed(
          () -> {
            operatorClimbEnabledPressed = true;
            if (operatorClimbEnabledPressed && driverClimbEnabledPressed) {
              climberSubsystem.toggleClimberEnabled();
              rgbSubsystem.climberEnabled();
            }
          }
        )
        .whenReleased(() -> operatorClimbEnabledPressed = false);
      buttons.driverEnableClimber
        .whenPressed(
          () -> {
            driverClimbEnabledPressed = true;
            if (operatorClimbEnabledPressed && driverClimbEnabledPressed) {
              climberSubsystem.toggleClimberEnabled();
              rgbSubsystem.climberEnabled();
            }
          }
        )
        .whenReleased(() -> driverClimbEnabledPressed = false);
    
      buttons.elevatorExtend.whenPressed(climberSubsystem::manualElevatorExtend);
      buttons.elevatorExtend.whenReleased(climberSubsystem::elevatorStop);
    
      buttons.elevatorRetract.whenPressed(climberSubsystem::manualElevatorRetract);
      buttons.elevatorRetract.whenReleased(climberSubsystem::elevatorStop);
      
      buttons.climbAuto.whenPressed(climberSubsystem::autoClimb);
      buttons.climbAuto.whenReleased(climberSubsystem::autoClimbReleased);
      buttons.resetClimbStuff.whenPressed(climberSubsystem::resetClimbStuff);
    
      buttons.autoClimbStopLeft.whenPressed(
        () -> {
          autoClimbStopLeftPressed = true;
          if (autoClimbStopLeftPressed && autoClimbStopRightPressed)
          {
            climberSubsystem.stopAutoClimb();
          }
        }
      )
      .whenReleased(() -> autoClimbStopLeftPressed = false);
      buttons.autoClimbStopRight.whenPressed(
        () -> {
          autoClimbStopRightPressed = true;
          if (autoClimbStopLeftPressed && autoClimbStopRightPressed)
          {
            climberSubsystem.stopAutoClimb();
          }
        }
      )
      .whenReleased(() -> autoClimbStopRightPressed = false);
    }

    buttons.rgb.whenPressed(() -> {
      rgbSubsystem.funnyButton();
    });
  }
}
