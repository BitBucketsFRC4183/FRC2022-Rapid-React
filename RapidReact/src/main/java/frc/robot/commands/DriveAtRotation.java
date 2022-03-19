// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.log.BucketLog;
import frc.robot.log.Changeable;
import frc.robot.log.Put;
import frc.robot.subsystem.DrivetrainSubsystem;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

/**
 * This class implements field centric swerve drive, with fixed rotational control. The robot
 * defaults to zero degree rotation. Pressing the Xbox POV buttons change the target angle.
 *
 * Inspired by Team 1684's comprehensive whitepaper on Swerve.
 * https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf and Team 2910's
 * 2021 competition robot code https://github.com/FRCTeam2910/2021CompetitionRobot
 */

public class DriveAtRotation extends CommandBase {

  private final DrivetrainSubsystem driveSubsystem;

  // input suppliers from joysticks

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private double snapAngleFinal;
  private static final Changeable<Double> kI = BucketLog.changeable(Put.DOUBLE, "kI", 0.2);

  // robot rotation in radians to hold while driving
  private double rotation_radians = 0.0;
  int[] angles = { -22, 68, 158, 248 - 360 };

  // PID controller to maintain fixed rotation.
  // use a ProfiledPIDController w/ Trapezoidal Profile
  // https://github.com/wpilibsuite/allwpilib/blob/2ad2d2ca9628ab4130135949c7cea3f71fd5d5b6/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/SwerveModule.java#L27-L34
  private ProfiledPIDController rotationController;

  /**
   *
   * DriveWithSetRotationCommand - field centric swerve drive, while holding a given robot rotation
   *
   * @param drivetrainSubsystem
   * @param translationXSupplier
   * @param translationYSupplier
   * @param rotationRadians
   *
   *
   */
  public DriveAtRotation(
    DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    Config config
  ) {
    this.driveSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    rotationController =
      new ProfiledPIDController(
        config.auto.pathThetaPID.getKP(),
        kI.currentValue(),
        config.auto.pathThetaPID.getKD(),
        new TrapezoidProfile.Constraints(
          driveSubsystem.getMaxAngularVelocity(),
          driveSubsystem.getMaxAngularVelocity() * 10.0
        )
      );
    snapAngleFinal = getSnapToAngle(driveSubsystem.getGyroAngle().getDegrees());

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(0.1, 0.1); // about 0.1 radians = 6 degrees, 6 deg/sec

    SmartDashboard.putNumber("RotationOutput", 0.0);
    SmartDashboard.putNumber("ThetaVError", rotationController.getVelocityError());
    SmartDashboard.putNumber("ThetaError", rotationController.getPositionError());
    SmartDashboard.putBoolean("ThetaAtTarget", rotationController.atGoal());
    SmartDashboard.putNumber("X", translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Y", translationYSupplier.getAsDouble());

    addRequirements(drivetrainSubsystem);
  }

  double getSnapToAngle(double currentAngleDegrees) {
    double currentAngleRadians = Math.toRadians(currentAngleDegrees);
    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < -Math.PI) {
      currentAngleRadiansMod += Math.PI;
    }
    double referenceAngleRadians = getReferenceAngle(currentAngleRadiansMod);

    // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (Math.toDegrees(adjustedReferenceAngleRadians) > 180) {
      adjustedReferenceAngleRadians -= Math.toRadians(360);
    } else if (Math.toDegrees(adjustedReferenceAngleRadians) < -180) {
      adjustedReferenceAngleRadians += Math.toRadians(360);
    }
    return adjustedReferenceAngleRadians;
  }

  double getSmallAngleDifference(double a1, double a2)
  {
   return Math.min((360) - Math.abs(a1-a2), Math.abs(a1-a2));

  }
  double getReferenceAngle(double currentAngleRadians) {
    double referenceAngleRadians = Arrays
      .stream(angles)
      .mapToObj(x -> (Integer) x)
      .sorted(
        (a1, a2) ->
          (int) (
            Math.abs(a1 - Math.toDegrees(currentAngleRadians)) - Math.abs(a2 - Math.toDegrees(currentAngleRadians))
          )
      )
      .findFirst()
      .get();
    referenceAngleRadians = Math.toRadians(referenceAngleRadians);
    return referenceAngleRadians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.setGoal(rotation_radians);
    SmartDashboard.putNumber("DriveWithRationAngle", rotation_radians);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.toRadians(snapAngleFinal) != rotation_radians) {
      // only reset PDI if target changes
      SmartDashboard.putNumber("TargetAngle", snapAngleFinal);

      rotation_radians = Math.toRadians(snapAngleFinal);
    }

    double rotationOutput = rotationController.calculate(driveSubsystem.getGyroAngle().getRadians(), rotation_radians);
    SmartDashboard.putNumber("TargetAngle", Math.toDegrees(rotation_radians));
    SmartDashboard.putNumber("RotationOutput", 0.0);
    SmartDashboard.putNumber("ThetaVError", rotationController.getVelocityError());
    SmartDashboard.putNumber("ThetaError", rotationController.getPositionError());
    SmartDashboard.putBoolean("ThetaAtTarget", rotationController.atGoal());
    SmartDashboard.putNumber("X", translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Y", translationYSupplier.getAsDouble());

    if (Math.abs(rotationOutput) < 0.05) {
      rotationOutput = 0.0;
    }

    driveSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        rotationOutput,
        driveSubsystem.getGyroAngle()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
