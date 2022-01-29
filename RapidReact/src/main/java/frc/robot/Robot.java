// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.log.LogTestSubsystem;
import frc.robot.simulator.SetModeTestSubsystem;
import frc.robot.simulator.SimulatorTestSubsystem;
import frc.robot.subsystem.*;
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

  private Buttons buttons;
  private Config config;

  private final List<BitBucketsSubsystem> robotSubsystems = new ArrayList<>();

  private AutonomousSubsystem autonomousSubsystem;
  private DrivetrainSubsystem drivetrainSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Field2d field;

  public static enum BitBucketsTrajectory {
    FarLeft,
    NearRight,
    PathPlanner,
  }

  private static final SendableChooser<BitBucketsTrajectory> trajectoryChooser = new SendableChooser<>();

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

    trajectoryChooser.setDefaultOption("Far Left", BitBucketsTrajectory.FarLeft);
    trajectoryChooser.addOption("Near Right", BitBucketsTrajectory.NearRight);
    trajectoryChooser.addOption("PathPlanner", BitBucketsTrajectory.PathPlanner);
    SmartDashboard.putData("Trajectory Chooser", trajectoryChooser);

    // Add Subsystems Here
    this.robotSubsystems.add(autonomousSubsystem = new AutonomousSubsystem(this.config));
    if (config.enableDriveSubsystem) {
      this.robotSubsystems.add(drivetrainSubsystem = new DrivetrainSubsystem(this.config));
    }
    this.robotSubsystems.add(intakeSubsystem = new IntakeSubsystem(this.config));
    this.robotSubsystems.add(shooterSubsystem = new ShooterSubsystem(this.config));

    // create a new field to update
    SmartDashboard.putData("Field", field);

    autonomousSubsystem.field = field;
    if (config.enableDriveSubsystem) {
      drivetrainSubsystem.field = field;

      drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
          drivetrainSubsystem,
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveForward)),
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveStrafe)),
          () -> -MathUtils.modifyAxis(buttons.driverControl.getRawAxis(buttons.swerveRotation))
        )
      );
    }

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
    drivetrainSubsystem.logger().logString(LogLevel.GENERAL, "info", "auton started");
    switch (trajectoryChooser.getSelected()) {
      case FarLeft:
        drivetrainSubsystem.setOdometry(config.auto.farLeftStart);
        autonomousSubsystem.setTrajectory(config.auto.farLeftStartTrajectory);
        break;
      case NearRight:
        drivetrainSubsystem.setOdometry(config.auto.nearRightStart);
        autonomousSubsystem.setTrajectory(config.auto.nearRightStartTrajectory);
        break;
      case PathPlanner:
        FollowTrajectoryCommand c = new FollowTrajectoryCommand("Path", this.drivetrainSubsystem);
        drivetrainSubsystem.setOdometry(c.getTrajectory().getInitialPose());
        c.schedule();
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    drivetrainSubsystem.logger().logString(LogLevel.GENERAL, "info", "still in auton!!");
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

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
      buttons.zeroGyroscope.whenPressed(drivetrainSubsystem::zeroGyroscope);
    }
    //Intake buttons
    buttons.intake.whenPressed(intakeSubsystem::spinForward);
    buttons.outtake.whenPressed(intakeSubsystem::spinBackward);
    buttons.intake.whenReleased(intakeSubsystem::stopSpin);
    buttons.outtake.whenReleased(intakeSubsystem::stopSpin);

    //Shooter BUttons
    buttons.hubShoot.whenPressed(shooterSubsystem::shootTop);
    buttons.hubShoot.whenReleased(shooterSubsystem::stopShoot);

    buttons.lowShoot.whenPressed(shooterSubsystem::shootLow);
    buttons.lowShoot.whenReleased(shooterSubsystem::stopShoot);

    buttons.tarmacShoot.whenPressed(
      () -> {
        shooterSubsystem.shootTarmac();
        drivetrainSubsystem.orient();
      }
    );
    buttons.tarmacShoot.whenReleased(shooterSubsystem::stopShoot);
  }
}
