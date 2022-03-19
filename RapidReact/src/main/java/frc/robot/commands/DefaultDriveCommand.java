package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;
import frc.robot.subsystem.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

  private static final SendableChooser<String> orientationChooser = new SendableChooser<>();
  private final Loggable<String> xWheelLoggable = BucketLog.loggable(Put.STRING, "drivetrain/xWheel");

  private final DrivetrainSubsystem driveSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter limiterX = new SlewRateLimiter(2);
  private final SlewRateLimiter limiterY = new SlewRateLimiter(2);

  public DefaultDriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier
  ) {
    this.driveSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    // Control Orientation Chooser
    orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
    orientationChooser.addOption("Robot Oriented", "Robot Oriented");
    SmartDashboard.putData("Orientation Chooser", orientationChooser);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement

    double xOutput = limiterX.calculate(translationXSupplier.getAsDouble()) * driveSubsystem.getMaxVelocity();
    double yOutput = limiterY.calculate(translationYSupplier.getAsDouble()) * driveSubsystem.getMaxVelocity();
    double rotationOutput = rotationSupplier.getAsDouble() * driveSubsystem.getMaxAngularVelocity();

    switch (orientationChooser.getSelected()) {
      case "Field Oriented":
        if (xOutput == 0 && yOutput == 0 && rotationOutput == 0) {
          driveSubsystem.stopSticky();
          xWheelLoggable.log(LogLevel.DEBUG, "xWheel active.");
        } else {
          driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, rotationOutput, driveSubsystem.getGyroAngle())
          );
          xWheelLoggable.log(LogLevel.DEBUG, "xWheel inactive.");
        }
        break;
      case "Robot Oriented":
        if (xOutput == 0 && yOutput == 0 && rotationOutput == 0) {
          driveSubsystem.stopSticky();
          xWheelLoggable.log(LogLevel.DEBUG, "xWheel active.");
        } else {
          ChassisSpeeds robotOrient = new ChassisSpeeds(xOutput, yOutput, rotationOutput);
          xWheelLoggable.log(LogLevel.DEBUG, "xWheel inactive.");
          driveSubsystem.drive(robotOrient);
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
